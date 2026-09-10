#include "nuway_planning/collision_checker.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <nuway_common/agents.h>
#include <nuway_common/geometry.h>
#include <nuway_common/trajectory.h>

namespace nuway_planning {
namespace {

using nuway_common::AgentState;
using nuway_common::kPi;
using nuway_common::PredictionSet;
using nuway_common::SE2;
using nuway_common::Trajectory;

// Segment intersection (proper or touching) for the brute-force overlap.
bool SegmentsIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                       const Eigen::Vector2d& q1, const Eigen::Vector2d& q2) {
  const auto cross = [](const Eigen::Vector2d& u, const Eigen::Vector2d& v) {
    return (u.x() * v.y()) - (u.y() * v.x());
  };
  const double d1 = cross(q2 - q1, p1 - q1);
  const double d2 = cross(q2 - q1, p2 - q1);
  const double d3 = cross(p2 - p1, q1 - p1);
  const double d4 = cross(p2 - p1, q2 - p1);
  return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
}

// Point in a convex polygon (counter-clockwise corners).
bool PointInside(const std::array<Eigen::Vector2d, 4>& poly,
                 const Eigen::Vector2d& p) {
  for (std::size_t i = 0; i < poly.size(); ++i) {
    const Eigen::Vector2d& a = poly[i];
    const Eigen::Vector2d& b = poly[(i + 1) % poly.size()];
    const Eigen::Vector2d e = b - a;
    const Eigen::Vector2d v = p - a;
    if ((e.x() * v.y()) - (e.y() * v.x()) < 0.0) {
      return false;
    }
  }
  return true;
}

// Brute force: a corner inside the other box, or two edges crossing.
bool BruteOverlap(const OrientedBox& a, const OrientedBox& b) {
  const std::array<Eigen::Vector2d, 4> ca = a.Corners();
  const std::array<Eigen::Vector2d, 4> cb = b.Corners();
  for (const Eigen::Vector2d& p : ca) {
    if (PointInside(cb, p)) {
      return true;
    }
  }
  for (const Eigen::Vector2d& p : cb) {
    if (PointInside(ca, p)) {
      return true;
    }
  }
  for (std::size_t i = 0; i < 4; ++i) {
    for (std::size_t j = 0; j < 4; ++j) {
      if (SegmentsIntersect(ca[i], ca[(i + 1) % 4], cb[j], cb[(j + 1) % 4])) {
        return true;
      }
    }
  }
  return false;
}

TEST(CollisionCheckerTest, SatMatchesBruteForceOnRandomBoxes) {
  // NOLINTNEXTLINE(bugprone-random-generator-seed): reproducible fixture
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> pos(-6.0, 6.0);
  std::uniform_real_distribution<double> yaw(-kPi, kPi);
  std::uniform_real_distribution<double> len(0.5, 6.0);
  int overlaps = 0;
  for (int i = 0; i < 5000; ++i) {
    OrientedBox a;
    a.center = Eigen::Vector2d(pos(rng), pos(rng));
    a.yaw_rad = yaw(rng);
    a.length_m = len(rng);
    a.width_m = len(rng);
    OrientedBox b;
    b.center = Eigen::Vector2d(pos(rng), pos(rng));
    b.yaw_rad = yaw(rng);
    b.length_m = len(rng);
    b.width_m = len(rng);
    const bool expected = BruteOverlap(a, b);
    overlaps += expected ? 1 : 0;
    EXPECT_EQ(BoxesOverlap(a, b), expected) << "pair " << i;
    EXPECT_EQ(BoxesOverlap(b, a), expected) << "pair " << i;
  }
  EXPECT_GT(overlaps, 500);  // the sample exercises both outcomes
  EXPECT_LT(overlaps, 4500);
}

TEST(CollisionCheckerTest, CornersAndDiscDistance) {
  OrientedBox box;
  box.center = Eigen::Vector2d(1.0, 2.0);
  box.yaw_rad = kPi / 2.0;
  box.length_m = 4.0;
  box.width_m = 2.0;
  const std::array<Eigen::Vector2d, 4> c = box.Corners();
  EXPECT_NEAR(c[0].x(), 0.0, 1e-12);  // front-left: forward is +y, left -x
  EXPECT_NEAR(c[0].y(), 4.0, 1e-12);
  EXPECT_NEAR(c[2].x(), 2.0, 1e-12);
  EXPECT_NEAR(c[2].y(), 0.0, 1e-12);
  // Two 4 x 2 boxes nose to tail 10 m apart along x: the disc gap is
  // 10 - 2 * (4/3) - 2 * sqrt((4/6)^2 + 1) = 4.93 (the discs bulge past the
  // rectangle ends by 0.2 m each), and 0 once they touch.
  OrientedBox a;
  a.length_m = 4.0;
  a.width_m = 2.0;
  OrientedBox b = a;
  b.center = Eigen::Vector2d(10.0, 0.0);
  const double r = std::hypot(4.0 / 6.0, 1.0);
  EXPECT_NEAR(DiscDistance(a, b), 10.0 - (8.0 / 3.0) - (2.0 * r), 1e-9);
  b.center = Eigen::Vector2d(4.0, 0.0);
  EXPECT_NEAR(DiscDistance(a, b), 0.0, 1e-12);
}

// A straight 10 m/s trajectory along +x from the origin (rear axle).
Trajectory Straight(double v) {
  Trajectory t;
  for (int i = 0; i < nuway_common::kTrajectoryPoints; ++i) {
    nuway_common::TrajectoryPoint p;
    p.t = i * nuway_common::kTrajectoryDtS;
    p.x = v * p.t;
    p.v = v;
    t.push_back(p);
  }
  return t;
}

AgentState Car(std::uint32_t id, double x, double y, double vx) {
  AgentState a;
  a.id = id;
  a.class_id = nuway_common::AgentClass::kCar;
  a.pose = SE2{x, y, 0.0};
  a.length_m = 4.5;
  a.width_m = 2.0;
  a.vx_mps = vx;
  return a;
}

PredictionSet ConstVel(const std::vector<AgentState>& agents, int samples,
                       double slow_down_factor) {
  PredictionSet set;
  set.num_samples = samples;
  set.num_timesteps = 16;
  set.dt_s = 0.5;
  for (const AgentState& a : agents) {
    set.agent_ids.push_back(a.id);
  }
  for (int s = 0; s < samples; ++s) {
    set.sample_weight.push_back(s == 0 ? 0.75 : 0.25);
    // Sample 1 (when present) moves slower by the factor.
    const double f = s == 0 ? 1.0 : slow_down_factor;
    for (const AgentState& a : agents) {
      for (int t = 0; t < 16; ++t) {
        const double dt = 0.5 * (t + 1);
        set.xy.push_back(a.pose.x + (f * a.vx_mps * dt));
        set.xy.push_back(a.pose.y);
        set.yaw.push_back(0.0);
      }
    }
  }
  return set;
}

TEST(CollisionCheckerTest, StaticObstacleAheadCollidesAtTheRightTime) {
  const CollisionChecker checker{CollisionOptions{}};
  // A parked car 40 m ahead in the lane. The ego front (rear axle + 1.389
  // + 2.446 + 1.0 margin = 4.835 m) meets its inflated rear (40 - 2.25 -
  // 0.2 = 37.55 m) at s = 32.7 m -> 3.27 s at 10 m/s: first check time 3.5.
  const std::vector<AgentState> agents = {Car(1, 40.0, 0.0, 0.0)};
  const CollisionResult hit =
      checker.Check(Straight(10.0), agents, ConstVel(agents, 1, 1.0));
  EXPECT_TRUE(hit.collides());
  EXPECT_NEAR(hit.cost, 1.0, 1e-12);
  EXPECT_NEAR(hit.min_ttc_s, 3.5, 1e-12);
  ASSERT_EQ(hit.times_s.size(), 17U);
  ASSERT_EQ(hit.min_distance_m.size(), 17U);
  EXPECT_GT(hit.min_distance_m[0], 30.0);
  EXPECT_LT(hit.min_distance_m[6], hit.min_distance_m[0]);
  EXPECT_NEAR(hit.min_distance_m[8], 0.0, 1e-12);  // t = 4 s: on top of it
  // The same car parked in the next lane: never touched, distance 1.3 m
  // at the closest (3.5 - 1.0 - 0.92 minus the disc bulge).
  const std::vector<AgentState> beside = {Car(1, 40.0, 3.5, 0.0)};
  const CollisionResult miss =
      checker.Check(Straight(10.0), beside, ConstVel(beside, 1, 1.0));
  EXPECT_FALSE(miss.collides());
  EXPECT_EQ(miss.min_ttc_s, std::numeric_limits<double>::infinity());
  EXPECT_GT(miss.min_distance_m[8], 0.5);
  // An agent absent from the prediction set is treated as static.
  const CollisionResult absent =
      checker.Check(Straight(10.0), agents, PredictionSet{});
  EXPECT_TRUE(absent.collides());
  EXPECT_NEAR(absent.min_ttc_s, 3.5, 1e-12);
}

TEST(CollisionCheckerTest, SampleWeightsAndMovingAgents) {
  const CollisionChecker checker{CollisionOptions{}};
  // A lead 20 m ahead at 10 m/s: sample 0 keeps pace (no collision),
  // sample 1 (weight 0.25) slows to 2 m/s and is caught.
  const std::vector<AgentState> agents = {Car(1, 20.0, 0.0, 10.0)};
  const CollisionResult r =
      checker.Check(Straight(10.0), agents, ConstVel(agents, 2, 0.2));
  EXPECT_NEAR(r.cost, 0.25, 1e-12);
  // The gap closes at 8 m/s from 20 - 2.25 - 0.2 - 4.835 = 12.7 m: 1.6 s.
  EXPECT_NEAR(r.min_ttc_s, 2.0, 1e-12);
  // t = 0 uses the observed pose whatever the samples say.
  const std::vector<AgentState> on_top = {Car(1, 3.0, 0.0, 30.0)};
  const CollisionResult now =
      checker.Check(Straight(10.0), on_top, ConstVel(on_top, 1, 1.0));
  EXPECT_NEAR(now.min_ttc_s, 0.0, 1e-12);
  // An empty trajectory: nothing to check.
  const CollisionResult none = checker.Check(Trajectory{}, agents, {});
  EXPECT_FALSE(none.collides());
  EXPECT_EQ(none.times_s.size(), 17U);
}

}  // namespace
}  // namespace nuway_planning
