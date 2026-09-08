// Planar (SE2) and spatial (SE3) rigid-transform helpers, angle wrapping and
// the Eigen aligned-vector aliases every nuway package uses. ROS convention
// throughout: x forward, y left, z up, yaw counter-clockwise positive, radians.
// Mirrored by nuway_ml/common/geometry.py (parity-tested). Introduced in M0.
#ifndef NUWAY_COMMON_GEOMETRY_H_
#define NUWAY_COMMON_GEOMETRY_H_

#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace nuway_common {

constexpr double kPi = 3.14159265358979323846;

// Fixed-size vectorizable Eigen types need the aligned allocator inside
// std::vector; use these aliases instead of spelling it out.
using Vector2dList =
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;
using Vector3dList =
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
using Vector4dList =
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>;

// Wraps an angle in radians into (-pi, pi].
inline double WrapAngle(double angle_rad) {
  double wrapped = std::fmod(angle_rad + kPi, 2.0 * kPi);
  if (wrapped <= 0.0) {
    wrapped += 2.0 * kPi;
  }
  return wrapped - kPi;
}

// Planar pose: translation (x, y) in meters and heading yaw in radians.
struct SE2 {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// Returns a * b (apply b in a's frame, then a). The result's yaw is wrapped.
inline SE2 Compose(const SE2& a, const SE2& b) {
  const double cos_a = std::cos(a.yaw);
  const double sin_a = std::sin(a.yaw);
  return SE2{a.x + (cos_a * b.x) - (sin_a * b.y),
             a.y + (sin_a * b.x) + (cos_a * b.y), WrapAngle(a.yaw + b.yaw)};
}

// Returns a^-1.
inline SE2 Inverse(const SE2& a) {
  const double cos_a = std::cos(a.yaw);
  const double sin_a = std::sin(a.yaw);
  return SE2{-(cos_a * a.x) - (sin_a * a.y), (sin_a * a.x) - (cos_a * a.y),
             WrapAngle(-a.yaw)};
}

// Returns a^-1 * b: the pose of b expressed in a's frame.
inline SE2 Between(const SE2& a, const SE2& b) {
  return Compose(Inverse(a), b);
}

// Transforms a point expressed in the frame of `pose` into the parent frame.
inline Eigen::Vector2d Apply(const SE2& pose, const Eigen::Vector2d& point) {
  const double cos_a = std::cos(pose.yaw);
  const double sin_a = std::sin(pose.yaw);
  return Eigen::Vector2d{pose.x + (cos_a * point.x()) - (sin_a * point.y()),
                         pose.y + (sin_a * point.x()) + (cos_a * point.y())};
}

// Rotates a vector (no translation) from the frame of `pose` into the parent.
inline Eigen::Vector2d Rotate(const SE2& pose, const Eigen::Vector2d& vec) {
  const double cos_a = std::cos(pose.yaw);
  const double sin_a = std::sin(pose.yaw);
  return Eigen::Vector2d{(cos_a * vec.x()) - (sin_a * vec.y()),
                         (sin_a * vec.x()) + (cos_a * vec.y())};
}

// Spatial pose: translation in meters and a unit quaternion.
struct SE3 {
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
};

// Roll, pitch, yaw (radians, ROS convention: Z-Y-X intrinsic) to quaternion.
inline Eigen::Quaterniond RpyToQuaternion(double roll_rad, double pitch_rad,
                                          double yaw_rad) {
  const Eigen::AngleAxisd roll(roll_rad, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(pitch_rad, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(yaw_rad, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond quat = yaw * pitch * roll;
  quat.normalize();
  return quat;
}

// Quaternion to (roll, pitch, yaw) in radians, each wrapped to (-pi, pi].
inline Eigen::Vector3d QuaternionToRpy(const Eigen::Quaterniond& quat) {
  const Eigen::Quaterniond unit = quat.normalized();
  const double qw = unit.w();
  const double qx = unit.x();
  const double qy = unit.y();
  const double qz = unit.z();
  const double sinr_cosp = 2.0 * ((qw * qx) + (qy * qz));
  const double cosr_cosp = 1.0 - (2.0 * ((qx * qx) + (qy * qy)));
  const double roll = std::atan2(sinr_cosp, cosr_cosp);
  double sinp = 2.0 * ((qw * qy) - (qz * qx));
  sinp = std::max(-1.0, std::min(1.0, sinp));
  const double pitch = std::asin(sinp);
  const double siny_cosp = 2.0 * ((qw * qz) + (qx * qy));
  const double cosy_cosp = 1.0 - (2.0 * ((qy * qy) + (qz * qz)));
  const double yaw = std::atan2(siny_cosp, cosy_cosp);
  return Eigen::Vector3d{roll, pitch, yaw};
}

// Yaw-only quaternion (rotation about z).
inline Eigen::Quaterniond YawToQuaternion(double yaw_rad) {
  return RpyToQuaternion(0.0, 0.0, yaw_rad);
}

// Heading about z of a quaternion, in (-pi, pi].
inline double QuaternionToYaw(const Eigen::Quaterniond& quat) {
  return QuaternionToRpy(quat).z();
}

// Returns a * b.
inline SE3 Compose(const SE3& a, const SE3& b) {
  SE3 out;
  out.translation = a.translation + (a.rotation * b.translation);
  out.rotation = (a.rotation * b.rotation).normalized();
  return out;
}

// Returns a^-1.
inline SE3 Inverse(const SE3& a) {
  SE3 out;
  out.rotation = a.rotation.conjugate();
  out.translation = -(out.rotation * a.translation);
  return out;
}

// Transforms a point from the frame of `pose` into the parent frame.
inline Eigen::Vector3d Apply(const SE3& pose, const Eigen::Vector3d& point) {
  return pose.translation + (pose.rotation * point);
}

// Planar projection of an SE3 pose: (x, y, yaw).
inline SE2 ToSE2(const SE3& pose) {
  return SE2{pose.translation.x(), pose.translation.y(),
             QuaternionToYaw(pose.rotation)};
}

// Euclidean distance between two planar points.
inline double Distance(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
  return (a - b).norm();
}

}  // namespace nuway_common

#endif  // NUWAY_COMMON_GEOMETRY_H_
