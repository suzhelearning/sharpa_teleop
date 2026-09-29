#include "manus_pose.hpp"
#include <array>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void expect(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}
bool close(double actual, double expected) { return std::abs(actual - expected) < 1e-5; }
}

int main() {
    try {
        std::array<SkeletonNode, 25> nodes{};
        const Eigen::Quaternionf root(Eigen::AngleAxisf(0.7F, Eigen::Vector3f::UnitZ()));
        const Eigen::Quaternionf finger(Eigen::AngleAxisf(0.5F, Eigen::Vector3f::UnitX()));
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            const Eigen::Vector3f position = Eigen::Vector3f(1, 2, 3) + root * Eigen::Vector3f(i, 0, 0);
            const auto rotation = i == 0 ? root : root * finger;
            nodes[i].transform.position = {position.x(), position.y(), position.z()};
            nodes[i].transform.rotation = {rotation.w(), rotation.x(), rotation.y(), rotation.z()};
        }
        geometry_msgs::msg::PoseArray message;
        expect(manus_ros::transform_skeleton(nodes.data(), nodes.size(), message), "valid skeleton rejected");
        expect(message.poses.size() == 25, "retarget requires 25 poses");
        expect(close(message.poses[0].position.x, 0) && close(message.poses[0].position.z, 0), "wrist must be origin");
        expect(close(message.poses[1].position.z, 21), "thumb ordering or root/basis transform changed");
        expect(close(message.poses[5].position.z, 1), "index finger ordering changed");
        expect(close(message.poses[1].orientation.w, std::cos(0.25)), "relative orientation angle changed");
        expect(close(message.poses[1].orientation.z, std::sin(0.25)), "relative orientation basis changed");
        expect(!manus_ros::transform_skeleton(nodes.data(), 24, message), "short skeleton accepted");
        nodes[24].transform.rotation = {0, 0, 0, 0};
        expect(!manus_ros::transform_skeleton(nodes.data(), nodes.size(), message), "zero quaternion accepted");
        nodes[24].transform.rotation = {1, 0, 0, 0};
        nodes[0].transform.position.x = std::numeric_limits<float>::infinity();
        expect(!manus_ros::transform_skeleton(nodes.data(), nodes.size(), message), "non-finite wrist accepted");
        std::cout << "PASS native point ordering, root-relative coordinates, rotation and invalid frame rejection\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
