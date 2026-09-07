#include "nuway_common/geometry.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace nuway_common {
namespace {

constexpr double kTol = 1e-9;

TEST(GeometryTest, WrapAngleMapsIntoHalfOpenInterval) {
  EXPECT_NEAR(WrapAngle(0.0), 0.0, kTol);
  EXPECT_NEAR(WrapAngle(kPi), kPi, kTol);
  EXPECT_NEAR(WrapAngle(-kPi), kPi, kTol);
  EXPECT_NEAR(WrapAngle(3.0 * kPi), kPi, kTol);
  EXPECT_NEAR(WrapAngle((2.0 * kPi) + 0.5), 0.5, kTol);
  EXPECT_NEAR(WrapAngle((-2.0 * kPi) - 0.5), -0.5, kTol);
}

TEST(GeometryTest, ComposeThenBetweenRecoversRelativePose) {
  const SE2 a{1.0, 2.0, 0.3};
  const SE2 b{-0.5, 4.0, -2.0};
  const SE2 ab = Compose(a, b);
  const SE2 recovered = Between(a, ab);
  EXPECT_NEAR(recovered.x, b.x, kTol);
  EXPECT_NEAR(recovered.y, b.y, kTol);
  EXPECT_NEAR(recovered.yaw, b.yaw, kTol);
}

TEST(GeometryTest, InverseComposesToIdentity) {
  const SE2 a{3.0, -1.0, 2.5};
  const SE2 ident = Compose(a, Inverse(a));
  EXPECT_NEAR(ident.x, 0.0, kTol);
  EXPECT_NEAR(ident.y, 0.0, kTol);
  EXPECT_NEAR(ident.yaw, 0.0, kTol);
}

TEST(GeometryTest, ApplyRotatesCounterClockwise) {
  const SE2 pose{0.0, 0.0, kPi / 2.0};
  const Eigen::Vector2d out = Apply(pose, Eigen::Vector2d(1.0, 0.0));
  EXPECT_NEAR(out.x(), 0.0, kTol);
  EXPECT_NEAR(out.y(), 1.0, kTol);
}

TEST(GeometryTest, RpyQuaternionRoundTrip) {
  const double roll = 0.1;
  const double pitch = -0.2;
  const double yaw = 2.0;
  const Eigen::Vector3d rpy =
      QuaternionToRpy(RpyToQuaternion(roll, pitch, yaw));
  EXPECT_NEAR(rpy.x(), roll, kTol);
  EXPECT_NEAR(rpy.y(), pitch, kTol);
  EXPECT_NEAR(rpy.z(), yaw, kTol);
  EXPECT_NEAR(QuaternionToYaw(YawToQuaternion(-2.5)), -2.5, kTol);
}

TEST(GeometryTest, Se3ComposeInverseIsIdentity) {
  SE3 a;
  a.translation = Eigen::Vector3d(1.0, 2.0, 3.0);
  a.rotation = RpyToQuaternion(0.3, -0.4, 1.2);
  const SE3 ident = Compose(a, Inverse(a));
  EXPECT_NEAR(ident.translation.norm(), 0.0, kTol);
  EXPECT_NEAR(std::abs(ident.rotation.w()), 1.0, kTol);
  const Eigen::Vector3d p = Apply(a, Eigen::Vector3d(0.0, 0.0, 0.0));
  EXPECT_NEAR(p.x(), 1.0, kTol);
  const SE2 planar = ToSE2(a);
  EXPECT_NEAR(planar.x, 1.0, kTol);
  EXPECT_NEAR(planar.y, 2.0, kTol);
  EXPECT_NEAR(planar.yaw, 1.2, kTol);
}

}  // namespace
}  // namespace nuway_common
