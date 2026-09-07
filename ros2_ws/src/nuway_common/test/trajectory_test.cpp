#include "nuway_common/trajectory.hpp"

#include <gtest/gtest.h>

namespace nuway_common {
namespace {

TEST(TrajectoryTest, InterpolateIsLinearBetweenSamples) {
  Trajectory traj;
  for (int i = 0; i < 3; ++i) {
    TrajectoryPoint p;
    p.t = i * 1.0;
    p.x = i * 2.0;
    p.v = 10.0 - i;
    p.yaw = i * 0.1;
    traj.push_back(p);
  }
  const TrajectoryPoint mid = Interpolate(traj, 0.5);
  EXPECT_NEAR(mid.x, 1.0, 1e-9);
  EXPECT_NEAR(mid.v, 9.5, 1e-9);
  EXPECT_NEAR(mid.yaw, 0.05, 1e-9);
  EXPECT_NEAR(Interpolate(traj, -1.0).x, 0.0, 1e-9);
  EXPECT_NEAR(Interpolate(traj, 9.0).x, 4.0, 1e-9);
}

TEST(TrajectoryTest, InterpolateWrapsYawAcrossPi) {
  Trajectory traj;
  TrajectoryPoint p0;
  p0.t = 0.0;
  p0.yaw = kPi - 0.1;
  TrajectoryPoint p1;
  p1.t = 1.0;
  p1.yaw = -kPi + 0.1;
  traj.push_back(p0);
  traj.push_back(p1);
  EXPECT_NEAR(Interpolate(traj, 0.5).yaw, kPi, 1e-9);
}

TEST(TrajectoryTest, StopTrajectoryHasEightyOnePointsAtRest) {
  const Trajectory stop = StopTrajectory(SE2{1.0, 2.0, 0.5});
  ASSERT_EQ(static_cast<int>(stop.size()), kTrajectoryPoints);
  EXPECT_NEAR(stop.back().t, 8.0, 1e-9);
  EXPECT_NEAR(stop.back().x, 1.0, 1e-9);
  EXPECT_NEAR(stop.back().v, 0.0, 1e-9);
  const Trajectory re = Resample(stop, 0.0, 0.5, 5);
  ASSERT_EQ(re.size(), 5U);
  EXPECT_NEAR(re[4].t, 2.0, 1e-9);
}

}  // namespace
}  // namespace nuway_common
