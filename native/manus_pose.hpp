#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_array.hpp>
#include "ManusSDKTypes.h"

namespace manus_ros {

// Preserve the existing retarget input contract: wrist, then thumb through pinky.
inline constexpr std::array<std::size_t, 25> point_order{
    0, 21, 22, 23, 24, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};

inline bool normalized_rotation(const ManusQuaternion& source, Eigen::Quaternionf& result) {
    result = Eigen::Quaternionf(source.w, source.x, source.y, source.z);
    const float norm = result.norm();
    if (!result.coeffs().allFinite() || !std::isfinite(norm) || norm <= 0.0F) {
        return false;
    }
    result.coeffs() /= norm;
    return true;
}

// Output is root-relative, with the same -90 degree Y basis change used by retargeting.
// Reuse the message storage; callers publish only after the entire conversion succeeds.
inline bool transform_skeleton(const SkeletonNode* nodes, std::size_t count,
                               geometry_msgs::msg::PoseArray& message) {
    if (count < point_order.size()) {
        return false;
    }
    const auto& root = nodes[0].transform;
    const Eigen::Vector3f root_position(root.position.x, root.position.y, root.position.z);
    Eigen::Quaternionf root_rotation;
    if (!root_position.allFinite() || !normalized_rotation(root.rotation, root_rotation)) {
        return false;
    }
    const auto root_inverse = root_rotation.conjugate();
    const Eigen::Quaternionf basis(Eigen::AngleAxisf(-1.5707963267948966F, Eigen::Vector3f::UnitY()));
    const auto basis_inverse = basis.conjugate();
    message.poses.resize(point_order.size());
    for (std::size_t index = 0; index < point_order.size(); ++index) {
        const auto& transform = nodes[point_order[index]].transform;
        const Eigen::Vector3f position(transform.position.x, transform.position.y, transform.position.z);
        Eigen::Quaternionf rotation;
        if (!position.allFinite() || !normalized_rotation(transform.rotation, rotation)) {
            return false;
        }
        const Eigen::Vector3f relative_position = basis * (root_inverse * (position - root_position));
        Eigen::Quaternionf relative_rotation = basis * (root_inverse * rotation) * basis_inverse;
        relative_rotation.normalize();
        if (!relative_position.allFinite() || !relative_rotation.coeffs().allFinite()) {
            return false;
        }
        auto& pose = message.poses[index];
        pose.position.x = relative_position.x();
        pose.position.y = relative_position.y();
        pose.position.z = relative_position.z();
        pose.orientation.w = relative_rotation.w();
        pose.orientation.x = relative_rotation.x();
        pose.orientation.y = relative_rotation.y();
        pose.orientation.z = relative_rotation.z();
    }
    return true;
}

}  // namespace manus_ros
