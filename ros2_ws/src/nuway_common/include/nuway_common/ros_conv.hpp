// geometry_msgs <-> Eigen / SE2 / SE3 conversions for nodes (part of
// nuway_common_ros; mirrors nuway_carla_bridge/ros_conv.py). No arithmetic
// beyond field copies, so nothing here needs a parity test. Introduced in M0.
#ifndef NUWAY_COMMON_ROS_CONV_HPP_
#define NUWAY_COMMON_ROS_CONV_HPP_

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "nuway_common/geometry.hpp"

namespace nuway_common {

inline geometry_msgs::msg::Point PointMsg(const Eigen::Vector3d& p) {
  geometry_msgs::msg::Point out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}

inline Eigen::Vector3d PointFromMsg(const geometry_msgs::msg::Point& p) {
  return Eigen::Vector3d{p.x, p.y, p.z};
}

inline geometry_msgs::msg::Quaternion QuaternionMsg(
    const Eigen::Quaterniond& q) {
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

inline Eigen::Quaterniond QuaternionFromMsg(
    const geometry_msgs::msg::Quaternion& q) {
  return Eigen::Quaterniond{q.w, q.x, q.y, q.z};
}

inline geometry_msgs::msg::Pose PoseMsg(const SE3& pose) {
  geometry_msgs::msg::Pose out;
  out.position = PointMsg(pose.translation);
  out.orientation = QuaternionMsg(pose.rotation);
  return out;
}

inline SE3 SE3FromMsg(const geometry_msgs::msg::Pose& pose) {
  return SE3{PointFromMsg(pose.position), QuaternionFromMsg(pose.orientation)};
}

inline SE2 SE2FromMsg(const geometry_msgs::msg::Pose& pose) {
  return SE2{pose.position.x, pose.position.y,
             QuaternionToYaw(QuaternionFromMsg(pose.orientation))};
}

}  // namespace nuway_common

#endif  // NUWAY_COMMON_ROS_CONV_HPP_
