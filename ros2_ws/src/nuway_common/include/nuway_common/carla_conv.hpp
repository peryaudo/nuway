// CARLA (UE4, left-handed: x forward, y right, z up, yaw clockwise, degrees)
// <-> ROS REP-103 (right-handed, radians) conversion. This header and
// nuway_ml/common/carla_conv.py are the only code allowed to know CARLA's
// convention (docs/01_directory_structure.md Rules, docs/02_interfaces.md §1):
//   x_ros = x_c;  y_ros = -y_c;  z_ros = z_c
//   roll_ros = roll_c;  pitch_ros = -pitch_c;  yaw_ros = -yaw_c  (deg -> rad)
// Applies only to data read through the CARLA Python API; CARLA's native ROS 2
// topics are already ROS-convention. Introduced in M0; GNSS <-> map in M5.
#ifndef NUWAY_COMMON_CARLA_CONV_HPP_
#define NUWAY_COMMON_CARLA_CONV_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "nuway_common/geometry.hpp"

namespace nuway_common {

// A carla.Location: meters, CARLA convention.
struct CarlaLocation {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// A carla.Rotation: degrees, CARLA convention (pitch about y, yaw about z,
// roll about x; yaw clockwise-positive seen from above).
struct CarlaRotation {
  double pitch_deg = 0.0;
  double yaw_deg = 0.0;
  double roll_deg = 0.0;
};

constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

// CARLA location or free vector (velocity, acceleration) -> ROS vector.
inline Eigen::Vector3d LocationToRos(const CarlaLocation& loc) {
  return Eigen::Vector3d{loc.x, -loc.y, loc.z};
}

// ROS vector -> CARLA location.
inline CarlaLocation LocationFromRos(const Eigen::Vector3d& ros) {
  return CarlaLocation{ros.x(), -ros.y(), ros.z()};
}

// CARLA rotation (degrees) -> ROS (roll, pitch, yaw) radians, wrapped.
inline Eigen::Vector3d RotationToRos(const CarlaRotation& rot) {
  return Eigen::Vector3d{WrapAngle(rot.roll_deg * kDegToRad),
                         WrapAngle(-rot.pitch_deg * kDegToRad),
                         WrapAngle(-rot.yaw_deg * kDegToRad)};
}

// ROS (roll, pitch, yaw) radians -> CARLA rotation (degrees).
inline CarlaRotation RotationFromRos(const Eigen::Vector3d& rpy_rad) {
  return CarlaRotation{-rpy_rad.y() * kRadToDeg, -rpy_rad.z() * kRadToDeg,
                       rpy_rad.x() * kRadToDeg};
}

// CARLA yaw in degrees -> ROS yaw in radians.
inline double YawToRos(double yaw_deg) {
  return WrapAngle(-yaw_deg * kDegToRad);
}

// ROS yaw in radians -> CARLA yaw in degrees.
inline double YawFromRos(double yaw_rad) { return -yaw_rad * kRadToDeg; }

// carla.Transform -> ROS SE3.
inline SE3 TransformToRos(const CarlaLocation& loc, const CarlaRotation& rot) {
  const Eigen::Vector3d rpy = RotationToRos(rot);
  SE3 out;
  out.translation = LocationToRos(loc);
  out.rotation = RpyToQuaternion(rpy.x(), rpy.y(), rpy.z());
  return out;
}

// CARLA angular velocity (deg/s, CARLA axes) -> ROS rad/s.
inline Eigen::Vector3d AngularVelocityToRos(const Eigen::Vector3d& omega_deg) {
  return Eigen::Vector3d{omega_deg.x() * kDegToRad, -omega_deg.y() * kDegToRad,
                         -omega_deg.z() * kDegToRad};
}

}  // namespace nuway_common

#endif  // NUWAY_COMMON_CARLA_CONV_HPP_
