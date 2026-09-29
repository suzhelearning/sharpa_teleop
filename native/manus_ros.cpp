// Project-owned ROS adapter. MANUS SDK headers, libraries and calibration stay external.
#include "manus_pose.hpp"
#include "ManusSDK.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) { stop_requested = 1; }
constexpr auto invalid_glove = std::numeric_limits<std::uint32_t>::max();

void require_success(SDKReturnCode code, const char* operation) {
    if (code != SDKReturnCode_Success) {
        throw std::runtime_error(std::string(operation) + " failed: SDK code " + std::to_string(code));
    }
}

class ManusPublisher final : public rclcpp::Node {
public:
    ManusPublisher() : Node("manus_native") {
        calibration_dir_ = declare_parameter<std::string>("calibration_dir", "");
        if (calibration_dir_.empty()) {
            throw std::invalid_argument("calibration_dir must name the authorized calibration directory");
        }
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        for (std::size_t side = 0; side < sides_.size(); ++side) {
            const std::string name = side == 0 ? "left" : "right";
            sides_[side].publisher = create_publisher<geometry_msgs::msg::PoseArray>(
                "/manus/" + name + "/raw_poses", qos);
            sides_[side].message.header.frame_id = "manus_" + name + "_hand";
            sides_[side].message.poses.resize(manus_ros::point_order.size());
            sides_[side].calibration = read_calibration(name);
        }
        nodes_.reserve(manus_ros::point_order.size());
    }

    ~ManusPublisher() override { stop(); }

    void initialize() {
        require_success(CoreSdk_InitializeIntegrated(), "Initialize MANUS Integrated SDK");
        initialized_ = true;
        active_.store(this);
        require_success(CoreSdk_RegisterCallbackForLandscapeStream(&landscape_callback), "Register landscape callback");
        require_success(CoreSdk_RegisterCallbackForRawSkeletonStream(&skeleton_callback), "Register skeleton callback");
        CoordinateSystemVUH coordinates;
        CoordinateSystemVUH_Init(&coordinates);
        coordinates.handedness = Side_Right;
        coordinates.up = AxisPolarity_PositiveZ;
        coordinates.view = AxisView_XFromViewer;
        coordinates.unitScale = 1.0F;
        require_success(CoreSdk_InitializeCoordinateSystemWithVUH(coordinates, true), "Configure MANUS coordinates");
        accepting_.store(true);
        RCLCPP_INFO(get_logger(), "MANUS initialized; direct ROS raw pose publishers ready");
    }

    bool connect() {
        require_success(CoreSdk_LookForHosts(1, false), "Discover MANUS hosts");
        std::uint32_t count = 0;
        require_success(CoreSdk_GetNumberOfAvailableHostsFound(&count), "Read MANUS hosts");
        if (count == 0) {
            RCLCPP_WARN(get_logger(), "Waiting for MANUS Integrated host");
            return false;
        }
        std::vector<ManusHost> hosts(count);
        require_success(CoreSdk_GetAvailableHostsFound(hosts.data(), count), "Get MANUS hosts");
        const auto result = CoreSdk_ConnectToHost(hosts.front());
        if (result == SDKReturnCode_NotConnected) {
            return false;
        }
        require_success(result, "Connect MANUS host");
        const auto motion_result = CoreSdk_SetRawSkeletonHandMotion(HandMotion_None);
        if (motion_result != SDKReturnCode_Success) {
            RCLCPP_WARN(get_logger(), "Optional raw hand-motion setting failed: SDK=%d",
                        static_cast<int>(motion_result));
        }
        RCLCPP_INFO(get_logger(), "Connected to MANUS; publishing /manus/{left,right}/raw_poses without ZMQ");
        return true;
    }

    void calibrate_connected_gloves() {
        for (std::size_t index = 0; index < sides_.size(); ++index) {
            auto& side = sides_[index];
            const auto id = side.glove_id.load();
            if (id == invalid_glove) {
                side.calibration_attempted = invalid_glove;
                continue;
            }
            if (id == side.calibration_attempted || side.calibration.empty()) {
                continue;
            }
            side.calibration_attempted = id;
            SetGloveCalibrationReturnCode result = SetGloveCalibrationReturnCode_Error;
            const auto code = CoreSdk_SetGloveCalibration(
                id, side.calibration.data(), static_cast<std::uint32_t>(side.calibration.size()), &result);
            if (code != SDKReturnCode_Success || result != SetGloveCalibrationReturnCode_Success) {
                RCLCPP_ERROR(get_logger(), "Calibration failed for %s glove %u: SDK=%d, result=%d",
                             index == 0 ? "left" : "right", id, static_cast<int>(code), static_cast<int>(result));
            } else {
                RCLCPP_INFO(get_logger(), "Loaded calibration for %s glove %u", index == 0 ? "left" : "right", id);
            }
        }
    }

    void stop() noexcept {
        accepting_.store(false);
        if (initialized_) {
            const auto result = CoreSdk_ShutDown();
            if (result != SDKReturnCode_Success) {
                std::fprintf(stderr, "MANUS SDK shutdown failed: %d\n", static_cast<int>(result));
            }
            initialized_ = false;
        }
        active_.store(nullptr);
    }

private:
    struct Hand {
        std::atomic<std::uint32_t> glove_id{invalid_glove};
        std::uint32_t calibration_attempted = invalid_glove;
        std::vector<unsigned char> calibration;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher;
        geometry_msgs::msg::PoseArray message;
    };

    std::vector<unsigned char> read_calibration(const std::string& side) {
        const auto path = std::filesystem::path(calibration_dir_) / ("Calibration_" + side + ".mcal");
        std::ifstream stream(path, std::ios::binary | std::ios::ate);
        if (!stream) {
            RCLCPP_WARN(get_logger(), "No readable calibration file %s; retaining SDK calibration", path.c_str());
            return {};
        }
        const auto length = stream.tellg();
        if (length <= 0 || static_cast<std::uint64_t>(length) > std::numeric_limits<std::uint32_t>::max()) {
            throw std::runtime_error("Invalid calibration file length: " + path.string());
        }
        std::vector<unsigned char> bytes(static_cast<std::size_t>(length));
        stream.seekg(0);
        if (!stream.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()))) {
            throw std::runtime_error("Could not read calibration file: " + path.string());
        }
        return bytes;
    }

    static void landscape_callback(const Landscape* landscape) noexcept {
        auto* instance = active_.load();
        if (!instance || !landscape) {
            return;
        }
        std::array<std::uint32_t, 2> ids{invalid_glove, invalid_glove};
        const auto count = std::min<std::size_t>(landscape->gloveDevices.gloveCount, MAX_NUMBER_OF_GLOVES);
        for (std::size_t index = 0; index < count; ++index) {
            const auto& glove = landscape->gloveDevices.gloves[index];
            if (glove.side == Side_Left && ids[0] == invalid_glove) {
                ids[0] = glove.id;
            } else if (glove.side == Side_Right && ids[1] == invalid_glove) {
                ids[1] = glove.id;
            }
        }
        for (std::size_t side = 0; side < ids.size(); ++side) {
            instance->sides_[side].glove_id.store(ids[side]);
        }
    }

    static void skeleton_callback(const SkeletonStreamInfo* stream) noexcept {
        auto* instance = active_.load();
        if (!instance || !stream || !instance->accepting_.load()) {
            return;
        }
        try {
            instance->publish_skeletons(*stream, instance->now());
        } catch (const std::exception& error) {
            std::fprintf(stderr, "MANUS raw pose callback failed: %s\n", error.what());
        } catch (...) {
            std::fprintf(stderr, "MANUS raw pose callback failed\n");
        }
    }

    void publish_skeletons(const SkeletonStreamInfo& stream, const rclcpp::Time& stamp) {
        // SDK callback threads share the reusable node/message storage.
        std::lock_guard<std::mutex> lock(skeleton_mutex_);
        if (!accepting_.load() || !rclcpp::ok()) {
            return;
        }
        for (std::uint32_t index = 0; index < stream.skeletonsCount; ++index) {
            RawSkeletonInfo info{};
            if (CoreSdk_GetRawSkeletonInfo(index, &info) != SDKReturnCode_Success) {
                continue;
            }
            Hand* hand = nullptr;
            for (auto& side : sides_) {
                if (info.gloveId != invalid_glove && info.gloveId == side.glove_id.load()) {
                    hand = &side;
                    break;
                }
            }
            if (!hand || info.nodesCount < manus_ros::point_order.size()) {
                continue;
            }
            nodes_.resize(info.nodesCount);
            if (CoreSdk_GetRawSkeletonData(index, nodes_.data(), info.nodesCount) != SDKReturnCode_Success) {
                continue;
            }
            if (!manus_ros::transform_skeleton(nodes_.data(), nodes_.size(), hand->message)) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected invalid MANUS skeleton");
                continue;
            }
            hand->message.header.stamp = stamp;
            hand->publisher->publish(hand->message);
        }
    }

    inline static std::atomic<ManusPublisher*> active_{nullptr};
    std::array<Hand, 2> sides_;
    std::vector<SkeletonNode> nodes_;
    std::mutex skeleton_mutex_;
    std::string calibration_dir_;
    std::atomic<bool> accepting_{false};
    bool initialized_ = false;
};
}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    int result = 0;
    try {
        auto node = std::make_shared<ManusPublisher>();
        node->initialize();
        bool connected = false;
        while (!stop_requested && rclcpp::ok()) {
            if (!connected) {
                connected = node->connect();
            }
            if (connected) {
                node->calibrate_connected_gloves();
            }
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        node->stop();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "MANUS ROS publisher failed: %s\n", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
