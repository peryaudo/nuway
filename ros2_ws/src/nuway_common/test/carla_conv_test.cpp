#include "nuway_common/carla_conv.hpp"

#include <gtest/gtest.h>

namespace nuway_common {
namespace {

constexpr double kTol = 1e-9;

TEST(CarlaConvTest, LocationFlipsY) {
  const Eigen::Vector3d ros = LocationToRos(CarlaLocation{1.0, 2.0, 3.0});
  EXPECT_NEAR(ros.x(), 1.0, kTol);
  EXPECT_NEAR(ros.y(), -2.0, kTol);
  EXPECT_NEAR(ros.z(), 3.0, kTol);
  const CarlaLocation back = LocationFromRos(ros);
  EXPECT_NEAR(back.y, 2.0, kTol);
}

TEST(CarlaConvTest, YawClockwiseDegreesBecomesCounterClockwiseRadians) {
  // CARLA yaw 90 deg (turned right seen from above) is ROS yaw -pi/2.
  EXPECT_NEAR(YawToRos(90.0), -kPi / 2.0, kTol);
  EXPECT_NEAR(YawFromRos(-kPi / 2.0), 90.0, kTol);
  // Wrapping: CARLA -180 deg -> ROS +pi.
  EXPECT_NEAR(YawToRos(-180.0), kPi, kTol);
}

TEST(CarlaConvTest, RotationRoundTrip) {
  const CarlaRotation rot{10.0, -30.0, 5.0};
  const Eigen::Vector3d rpy = RotationToRos(rot);
  EXPECT_NEAR(rpy.x(), 5.0 * kDegToRad, kTol);
  EXPECT_NEAR(rpy.y(), -10.0 * kDegToRad, kTol);
  EXPECT_NEAR(rpy.z(), 30.0 * kDegToRad, kTol);
  const CarlaRotation back = RotationFromRos(rpy);
  EXPECT_NEAR(back.pitch_deg, rot.pitch_deg, kTol);
  EXPECT_NEAR(back.yaw_deg, rot.yaw_deg, kTol);
  EXPECT_NEAR(back.roll_deg, rot.roll_deg, kTol);
}

TEST(CarlaConvTest, TransformToRosRightMountedSensorHasNegativeY) {
  // A sensor 1.5 m to the CARLA right (y = +1.5) is at ROS y = -1.5
  // (docs/02_interfaces.md §1, M0 finding).
  const SE3 pose = TransformToRos(CarlaLocation{1.0, 1.5, 2.0},
                                  CarlaRotation{0.0, 90.0, 0.0});
  EXPECT_NEAR(pose.translation.y(), -1.5, kTol);
  EXPECT_NEAR(QuaternionToYaw(pose.rotation), -kPi / 2.0, kTol);
}

TEST(CarlaConvTest, AngularVelocityFlipsPitchAndYaw) {
  const Eigen::Vector3d omega =
      AngularVelocityToRos(Eigen::Vector3d(0.0, 0.0, 90.0));
  EXPECT_NEAR(omega.z(), -kPi / 2.0, kTol);
}

}  // namespace
}  // namespace nuway_common
