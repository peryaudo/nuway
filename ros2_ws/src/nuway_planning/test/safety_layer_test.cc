#include "nuway_planning/safety_layer.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include <nuway_common/agents.h>
#include <nuway_common/geometry.h>
#include <nuway_common/occupancy.h>
#include <nuway_common/tick.h>
#include <nuway_common/trajectory.h>

#include "nuway_planning/collision_checker.h"

namespace nuway_planning {
namespace {

using nuway_common::AgentState;
using nuway_common::SE2;
using nuway_common::Trajectory;
using nuway_common::TrajectoryPoint;

// A straight along +x from the origin at constant v.
Trajectory Straight(double v) {
  Trajectory t;
  for (int i = 0; i < nuway_common::kTrajectoryPoints; ++i) {
    TrajectoryPoint p;
    p.t = i * nuway_common::kTrajectoryDtS;
    p.x = v * p.t;
    p.v = v;
    t.push_back(p);
  }
  return t;
}

AgentState ParkedCar(double x, double y) {
  AgentState a;
  a.id = 1;
  a.class_id = nuway_common::AgentClass::kCar;
  a.pose = SE2{x, y, 0.0};
  a.length_m = 4.5;
  a.width_m = 2.0;
  return a;
}

SafetyOptions Options() {
  SafetyOptions o;
  o.collision.margin_lon_m = 2.0;
  o.collision.margin_lat_m = 0.8;
  o.collision.agent_margin_m = 0.4;
  return o;
}

SafetyInput Input(const Trajectory* traj, double v) {
  SafetyInput in;
  in.ego_pose = SE2{0.0, 0.0, 0.0};
  in.ego_speed_mps = v;
  in.trajectory = traj;
  in.source = "lattice";
  return in;
}

TEST(SafetyLayerTest, ControlOnlyTickRetimesTheTrajectory) {
  SafetyLayer layer{Options()};
  const Trajectory traj = Straight(10.0);
  SafetyInput in = Input(&traj, 10.0);
  in.trajectory_age_s = nuway_common::kTickDtS;
  const SafetyOutput out = layer.Step(in);
  EXPECT_FALSE(out.intervened);
  EXPECT_EQ(out.source, "lattice");
  ASSERT_EQ(out.trajectory.size(), 81U);
  for (std::size_t i = 0; i < 81; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    EXPECT_NEAR(out.trajectory[i].t, t, 1e-12);
    EXPECT_NEAR(out.trajectory[i].x, 10.0 * (t + 0.05), 1e-9);
    EXPECT_NEAR(out.trajectory[i].v, 10.0, 1e-12);
  }
  // On a planning tick (age 0) the trajectory passes unchanged.
  in.trajectory_age_s = 0.0;
  EXPECT_NEAR(layer.Step(in).trajectory.back().x, 80.0, 1e-9);
  // The no-input trajectory passes through as a stop at the pose.
  in.source = "none";
  const SafetyOutput none = layer.Step(in);
  EXPECT_EQ(none.source, "none");
  EXPECT_FALSE(none.intervened);
  EXPECT_NEAR(none.trajectory.back().x, 0.0, 1e-12);
  EXPECT_NEAR(none.trajectory.back().v, 0.0, 1e-12);
}

TEST(SafetyLayerTest, CollisionWithinTheHorizonBecomesAMaxDecelStop) {
  SafetyLayer layer{Options()};
  const Trajectory traj = Straight(10.0);
  // A car 20 m ahead: the front (rear axle + 1.39 + 2.45 + 2.0 = 5.8 m)
  // meets its inflated rear (20 - 2.25 - 0.4 = 17.35 m) at 1.15 s.
  const std::vector<AgentState> agents = {ParkedCar(20.0, 0.0)};
  SafetyInput in = Input(&traj, 10.0);
  in.agents = &agents;
  const SafetyOutput out = layer.Step(in);
  EXPECT_TRUE(out.intervened);
  EXPECT_EQ(out.reason, "collision");
  EXPECT_EQ(out.source, "fallback");
  ASSERT_EQ(out.trajectory.size(), 81U);
  // Constant 6 m/s^2 along the same path: at rest after 1.67 s at 8.33 m.
  EXPECT_NEAR(out.trajectory[5].v, 10.0 - (6.0 * 0.5), 1e-9);
  EXPECT_NEAR(out.trajectory[5].a, -6.0, 1e-9);
  EXPECT_NEAR(out.trajectory.back().v, 0.0, 1e-12);
  EXPECT_NEAR(out.trajectory.back().x, 100.0 / 12.0, 1e-9);
  EXPECT_NEAR(out.trajectory.back().y, 0.0, 1e-12);
  // The same car 50 m ahead is hit after the 3 s horizon: no intervention.
  const std::vector<AgentState> far = {ParkedCar(50.0, 0.0)};
  in.agents = &far;
  EXPECT_FALSE(layer.Step(in).intervened);
}

TEST(SafetyLayerTest, OccupiedCellsUnderTheFootprintStop) {
  SafetyLayer layer{Options()};
  const Trajectory traj = Straight(5.0);
  OccupancyView grid;
  grid.base_pose = SE2{0.0, 0.0, 0.0};
  grid.occupied.assign(static_cast<std::size_t>(grid.spec.height) *
                           static_cast<std::size_t>(grid.spec.width),
                       0.0F);
  // Occupied cells at x in [10, 12), y in [-1, 1) (base_link): reached
  // by the front bumper (rear axle + 3.8 m) at about 1.2 s.
  for (int x = 0; x < 4; ++x) {
    for (int y = 0; y < 4; ++y) {
      const std::size_t r = 120U + static_cast<std::size_t>(x);
      const std::size_t c = 98U + static_cast<std::size_t>(y);
      grid.occupied[(r * static_cast<std::size_t>(grid.spec.width)) + c] = 1.0F;
    }
  }
  SafetyInput in = Input(&traj, 5.0);
  in.occupancy = &grid;
  const SafetyOutput out = layer.Step(in);
  EXPECT_TRUE(out.intervened);
  EXPECT_EQ(out.reason, "occupancy");
  EXPECT_NEAR(out.trajectory.back().v, 0.0, 1e-12);
  // Below the threshold: no hit.
  for (float& v : grid.occupied) {
    v = v > 0.0F ? 0.5F : 0.0F;
  }
  EXPECT_FALSE(layer.Step(in).intervened);
}

TEST(SafetyLayerTest, DegradedPlannerHoldsTheGentlestClearStop) {
  SafetyLayer layer{Options()};
  const Trajectory traj = Straight(10.0);
  const SafetyInput in = Input(&traj, 10.0);
  EXPECT_FALSE(layer.Step(in).intervened);  // the last safe path
  // Degraded with a clear road: the gentle profile along that path.
  SafetyInput degraded = Input(nullptr, 10.0);
  degraded.planner_degraded = true;
  const SafetyOutput gentle = layer.Step(degraded);
  EXPECT_TRUE(gentle.intervened);
  EXPECT_EQ(gentle.reason, "degraded");
  EXPECT_EQ(gentle.source, "fallback");
  EXPECT_NEAR(gentle.trajectory[10].a, -1.5, 1e-9);
  EXPECT_NEAR(gentle.trajectory[10].v, 8.5, 1e-9);
  // Held on the next tick, shifted by one tick.
  const SafetyOutput held = layer.Step(degraded);
  EXPECT_NEAR(held.trajectory[0].v, 10.0 - (1.5 * 0.05), 1e-9);
  EXPECT_NEAR(held.trajectory[0].x,
              gentle.trajectory[0].x + (10.0 * 0.05) - (0.75 * 0.05 * 0.05),
              5e-3);  // re-timed by linear interpolation of 0.1 s samples
  // With an obstacle the gentle stop would hit: max decel instead.
  SafetyLayer other{Options()};
  EXPECT_FALSE(other.Step(in).intervened);
  const std::vector<AgentState> agents = {ParkedCar(25.0, 0.0)};
  degraded.agents = &agents;
  const SafetyOutput hard = other.Step(degraded);
  EXPECT_EQ(hard.reason, "degraded:hard");
  EXPECT_NEAR(hard.trajectory[10].a, -6.0, 1e-9);
  // No safe trajectory yet: the no-input stop at the pose.
  SafetyLayer fresh{Options()};
  const SafetyOutput none = fresh.Step(degraded);
  EXPECT_EQ(none.source, "none");
  EXPECT_NEAR(none.trajectory.back().v, 0.0, 1e-12);
  fresh.Reset();
}

TEST(SafetyLayerTest, LimitsAreClippedAndReintegrated) {
  SafetyLayer layer{Options()};
  Trajectory traj = Straight(5.0);
  for (TrajectoryPoint& p : traj) {
    p.kappa = 2.0;  // far past kappa_phys
    p.a = 5.0;      // past a_max
  }
  const SafetyInput in = Input(&traj, 5.0);
  const SafetyOutput out = layer.Step(in);
  EXPECT_TRUE(out.intervened);
  EXPECT_EQ(out.reason, "limits");
  EXPECT_EQ(out.source, "lattice");
  for (const TrajectoryPoint& p : out.trajectory) {
    EXPECT_NEAR(p.kappa, 0.96, 1e-12);
    EXPECT_NEAR(p.a, 3.0, 1e-12);
  }
  // Re-integrated: accelerating at 3 and turning at 0.96 rad/m (a full
  // circle per second at this speed, so look 0.3 s in).
  EXPECT_NEAR(out.trajectory[10].v, 5.0 + 3.0, 1e-9);
  EXPECT_GT(std::abs(out.trajectory[3].y), 0.3);
  EXPECT_NE(out.trajectory[3].yaw, 0.0);
}

TEST(SafetyLayerTest, StopAlongPathFollowsTheGeometry) {
  // A quarter circle of radius 20 m.
  Trajectory path;
  for (int i = 0; i <= 80; ++i) {
    const double theta = (nuway_common::kPi / 2.0) * i / 80.0;
    TrajectoryPoint p;
    p.t = 0.1 * i;
    p.x = 20.0 * std::sin(theta);
    p.y = 20.0 * (1.0 - std::cos(theta));
    p.yaw = theta;
    p.kappa = 0.05;
    p.v = 4.0;
    path.push_back(p);
  }
  const Trajectory stop = StopAlongPath(path, 8.0, 2.0);
  ASSERT_EQ(stop.size(), 81U);
  // At rest after 4 s, 16 m along the arc: on the circle.
  EXPECT_NEAR(stop[40].v, 0.0, 1e-12);
  const double r = std::hypot(stop[40].x, stop[40].y - 20.0);
  EXPECT_NEAR(r, 20.0, 0.05);
  EXPECT_NEAR(std::hypot(stop.back().x, stop.back().y - 20.0), 20.0, 0.05);
  EXPECT_NEAR(stop[40].yaw, 16.0 / 20.0, 0.02);
  // From rest: holds the first point.
  const Trajectory still = StopAlongPath(path, 0.0, 2.0);
  EXPECT_NEAR(still.back().x, 0.0, 1e-12);
  EXPECT_NEAR(Retime(path, 0.5)[0].x, path[5].x, 1e-9);
}

}  // namespace
}  // namespace nuway_planning
