#include "nuway_planning/lattice_sampler.h"

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <nuway_common/agents.h>
#include <nuway_common/frenet.h>
#include <nuway_common/geometry.h>
#include <nuway_common/trajectory.h>

#include "nuway_planning/behavior_fsm.h"
#include "nuway_planning/candidate.h"
#include "nuway_planning/route_line.h"
#include "nuway_planning/scene.h"

namespace nuway_planning {
namespace {

using nuway_common::FrenetState;
using nuway_common::kPi;

constexpr double kLimit = 13.9;

// A straight of `length` m at 0.5 m spacing, one lane, 1.75 m bounds.
RouteLine Straight(double length) {
  nuway_common::Vector2dList points;
  const int n = static_cast<int>(std::lround(length / 0.5));
  for (int i = 0; i <= n; ++i) {
    points.emplace_back(0.5 * i, 0.0);
  }
  return RouteLine(nuway_common::ReferenceLine::FromPoints(points), {1U},
                   {kLimit}, {1.75}, {1.75}, length - 50.0);
}

// A circle of radius r (kappa = 1 / r) over three quarters of a turn.
RouteLine Circle(double r) {
  nuway_common::Vector2dList points;
  const double step = 0.25;
  const int n = static_cast<int>(1.5 * kPi * r / step);
  for (int i = 0; i <= n; ++i) {
    const double theta = i * step / r;
    points.emplace_back(r * std::cos(theta), r * std::sin(theta));
  }
  return RouteLine(nuway_common::ReferenceLine::FromPoints(points), {1U},
                   {kLimit}, {1.75}, {1.75}, std::nullopt);
}

FrenetState Ego(double s, double v, double d = 0.0) {
  FrenetState f;
  f.s = s;
  f.s_dot = v;
  f.d = d;
  return f;
}

BehaviorOutput Decision(Longitudinal longitudinal, double target_speed,
                        double stop_s = -1.0) {
  BehaviorOutput out;
  out.longitudinal = longitudinal;
  out.target_speed_mps = target_speed;
  out.stop_s = stop_s;
  out.reason = "test";
  return out;
}

std::size_t CountInjected(const std::vector<Candidate>& set) {
  std::size_t n = 0;
  for (const Candidate& c : set) {
    n += c.injected ? 1 : 0;
  }
  return n;
}

std::size_t CountFeasibleNormal(const std::vector<Candidate>& set) {
  std::size_t n = 0;
  for (const Candidate& c : set) {
    n += (!c.injected && c.feasible()) ? 1 : 0;
  }
  return n;
}

TEST(LatticeSamplerTest, FreeRoadSamplesTheFullLattice) {
  const RouteLine route = Straight(300.0);
  SceneInput in;
  in.route = &route;
  const LatticeSampler sampler{LatticeOptions{}, LatticeLimits{}};
  // 10 m/s: ds = {30, 35, 50} (3 v = 30 lifts the 20), 5 offsets, 4 speeds
  // (7, 8.5, 10, 11.5 all under the limit), 3 horizons: 15 x 12 = 180.
  std::vector<Candidate> set =
      sampler.Sample(in, Ego(20.0, 10.0), Decision(Longitudinal::kFree, 10.0));
  EXPECT_EQ(set.size(), 182U);
  EXPECT_EQ(CountInjected(set), 2U);
  for (std::size_t i = 0; i < set.size(); ++i) {
    EXPECT_EQ(set[i].id, i);
    EXPECT_EQ(set[i].trajectory.size(), 81U);
    EXPECT_EQ(set[i].frenet.size(), 81U);
    EXPECT_NEAR(set[i].trajectory.back().t, 8.0, 1e-9);
  }
  // The set starts at the ego state.
  EXPECT_NEAR(set[0].trajectory[0].x, 20.0, 1e-6);
  EXPECT_NEAR(set[0].trajectory[0].v, 10.0, 1e-6);
  sampler.Filter(route, &set);
  EXPECT_GT(CountFeasibleNormal(set), 100U);
  // At the limit the +1.5 target clips onto the limit and is dropped (3
  // speeds), and 3 v = 41.7 lifts two of the ds onto each other (2 ends).
  const std::vector<Candidate> at_limit = sampler.Sample(
      in, Ego(20.0, kLimit), Decision(Longitudinal::kFree, kLimit));
  EXPECT_EQ(at_limit.size(), (10U * 9U) + 2U);
}

TEST(LatticeSamplerTest, FeasibleCandidatesRespectTheLimits) {
  const RouteLine route = Straight(300.0);
  SceneInput in;
  in.route = &route;
  LatticeLimits limits;
  limits.a_max_mps2 = 2.0;
  limits.a_min_mps2 = -4.0;
  const LatticeSampler sampler{LatticeOptions{}, limits};
  std::vector<Candidate> set = sampler.Sample(
      in, Ego(20.0, 8.0, 0.4), Decision(Longitudinal::kStop, 0.0, 45.0));
  sampler.Filter(route, &set);
  std::size_t rejected = 0;
  for (const Candidate& c : set) {
    if (c.injected) {
      continue;
    }
    if (!c.feasible()) {
      ++rejected;
      continue;
    }
    for (std::size_t i = 0; i < c.trajectory.size(); ++i) {
      const nuway_common::TrajectoryPoint& p = c.trajectory[i];
      EXPECT_LE(p.a, 2.0 + 1e-6);
      EXPECT_GE(p.a, -4.0 - 1e-6);
      EXPECT_LE(std::abs(p.kappa), 0.96 + 1e-6);
      EXPECT_LE(p.v * p.v * std::abs(p.kappa), 4.0 + 1e-6);
      EXPECT_LE(std::abs(c.frenet[i].d), 1.75 - (0.5 * 1.837) + 1e-6);
      EXPECT_GE(p.v, -0.05);
    }
  }
  // The 1.0 m offsets leave the drivable band (1.75 - 0.92 = 0.83 m), and
  // the 3 s stop from 8 m/s over 25 m exceeds 4 m/s^2: something rejects.
  EXPECT_GT(rejected, 0U);
  EXPECT_GT(CountFeasibleNormal(set), 0U);
  // A stopping candidate ends at rest at the stop line.
  for (const Candidate& c : set) {
    if (c.speed_kind == SpeedKind::kStop && c.feasible()) {
      EXPECT_NEAR(c.frenet.back().s, 45.0, 1e-6);
      EXPECT_NEAR(c.trajectory.back().v, 0.0, 1e-6);
    }
  }
}

TEST(LatticeSamplerTest, TightCornerKeepsItsNormalCandidates) {
  // R = 2.4 m: the Town03 junction corner of M1 §3.3 (kappa = 0.42).
  const RouteLine route = Circle(2.4);
  SceneInput in;
  in.route = &route;
  const LatticeSampler physical{LatticeOptions{}, LatticeLimits{}};
  std::vector<Candidate> set =
      physical.Sample(in, Ego(1.0, 2.0), Decision(Longitudinal::kFree, 2.0));
  physical.Filter(route, &set);
  EXPECT_GT(CountFeasibleNormal(set), 0U);
  // With the YAML's comfort figure 0.18 every normal candidate would go.
  LatticeLimits comfort;
  comfort.kappa_phys = 0.18;
  const LatticeSampler yaml{LatticeOptions{}, comfort};
  std::vector<Candidate> rejected =
      yaml.Sample(in, Ego(1.0, 2.0), Decision(Longitudinal::kFree, 2.0));
  yaml.Filter(route, &rejected);
  EXPECT_EQ(CountFeasibleNormal(rejected), 0U);
  EXPECT_EQ(CountInjected(rejected), 2U);
}

TEST(LatticeSamplerTest, PathEndIsClippedAtTheLineEnd) {
  const RouteLine route = Straight(100.0);
  SceneInput in;
  in.route = &route;
  const LatticeSampler sampler{LatticeOptions{}, LatticeLimits{}};
  // 30 m from the end at 5 m/s: ds = {20, 35, 50} -> s_f = {90, 100, 100},
  // the duplicate dropped: 5 x 2 paths.
  const std::vector<Candidate> set =
      sampler.Sample(in, Ego(70.0, 5.0), Decision(Longitudinal::kFree, 5.0));
  EXPECT_EQ(set.size(), (10U * 12U) + 2U);
  for (const Candidate& c : set) {
    EXPECT_LE(c.s_f_m, 100.0 + 1e-9);
    EXPECT_LE(c.frenet.back().s, 100.0 + 1e-9);
  }
  // 3 m from the end: no lateral fit, the current d is held: one path.
  const std::vector<Candidate> hold = sampler.Sample(
      in, Ego(97.0, 5.0, 0.3), Decision(Longitudinal::kFree, 5.0));
  EXPECT_EQ(hold.size(), 12U + 2U);
  for (const Candidate& c : hold) {
    for (const FrenetState& f : c.frenet) {
      EXPECT_NEAR(f.d, 0.3, 1e-9);
    }
    // A candidate reaching the line end rests at the last sample.
    EXPECT_LE(c.frenet.back().s, 100.0 + 1e-9);
    if (c.frenet.back().s > 100.0 - 1e-9) {
      EXPECT_NEAR(c.trajectory.back().v, 0.0, 1e-9);
    }
  }
}

TEST(LatticeSamplerTest, InjectedStopsAreAlwaysPresentAndNeverRejected) {
  const RouteLine route = Straight(300.0);
  SceneInput in;
  in.route = &route;
  LatticeLimits absurd;
  absurd.kappa_phys = 0.001;  // would reject anything but a straight hold
  absurd.a_min_mps2 = -0.1;
  const LatticeSampler sampler{LatticeOptions{}, absurd};
  const FrenetState ego = Ego(20.0, 10.0, 0.5);
  const std::vector<BehaviorOutput> decisions = {
      Decision(Longitudinal::kFree, 10.0),
      Decision(Longitudinal::kFollow, 8.0),
      Decision(Longitudinal::kYield, 0.0, 60.0),
      Decision(Longitudinal::kStop, 0.0, 60.0),
  };
  for (const BehaviorOutput& decision : decisions) {
    std::vector<Candidate> set = sampler.Sample(in, ego, decision);
    sampler.Filter(route, &set);
    ASSERT_GE(set.size(), 2U);
    EXPECT_EQ(CountInjected(set), 2U);
    const Candidate& gentle = set[set.size() - 2];
    const Candidate& hard = set.back();
    EXPECT_TRUE(gentle.injected);
    EXPECT_TRUE(gentle.feasible());
    EXPECT_EQ(gentle.source, "stop");
    EXPECT_EQ(gentle.speed_kind, SpeedKind::kGentleStop);
    EXPECT_NEAR(gentle.frenet.back().s, 20.0 + (100.0 / 3.0), 1e-6);
    EXPECT_NEAR(gentle.trajectory.back().v, 0.0, 1e-6);
    EXPECT_NEAR(gentle.frenet.back().d, 0.0, 1e-6);  // back on the centre
    EXPECT_TRUE(hard.injected);
    EXPECT_TRUE(hard.feasible());
    EXPECT_EQ(hard.speed_kind, SpeedKind::kHardStop);
    // Constant 0.1 m/s^2 from 10 m/s does not stop within 8 s here; with
    // the real -6 m/s^2 it rests after 1.67 s at s + 8.33.
    EXPECT_GT(hard.trajectory.back().v, 0.0);
  }
  const LatticeSampler real{LatticeOptions{}, LatticeLimits{}};
  const std::vector<Candidate> pair = real.SampleInjected(in, ego, 7);
  ASSERT_EQ(pair.size(), 2U);
  EXPECT_EQ(pair[0].id, 7U);
  EXPECT_EQ(pair[1].id, 8U);
  EXPECT_NEAR(pair[1].frenet.back().s, 20.0 + (100.0 / 12.0), 1e-6);
  EXPECT_NEAR(pair[1].trajectory.back().v, 0.0, 1e-6);
  EXPECT_NEAR(pair[1].trajectory[1].a, -6.0, 1e-3);  // d' != 0: a != s_ddot
  // Nearly stopped: both hold the pose.
  const std::vector<Candidate> held =
      real.SampleInjected(in, Ego(20.0, 0.1, 0.4), 0);
  for (const Candidate& c : held) {
    EXPECT_NEAR(c.frenet.back().s, 20.0, 0.01);
    EXPECT_NEAR(c.frenet.back().d, 0.4, 1e-9);
  }
}

TEST(LatticeSamplerTest, FollowAddsGapKeepingAndEgoProjects) {
  const RouteLine route = Straight(300.0);
  SceneInput in;
  in.route = &route;
  nuway_common::AgentState lead;
  lead.id = 5;
  lead.pose = nuway_common::SE2{60.0, 0.0, 0.0};
  lead.length_m = 4.5;
  lead.vx_mps = 6.0;
  in.agents = {lead};
  BehaviorOutput decision = Decision(Longitudinal::kFollow, 7.0);
  decision.lead_agent_id = 5;
  const LatticeSampler sampler{LatticeOptions{}, LatticeLimits{}};
  const std::vector<Candidate> set =
      sampler.Sample(in, Ego(20.0, 8.0), decision);
  std::size_t gap = 0;
  for (const Candidate& c : set) {
    if (c.speed_kind == SpeedKind::kGap) {
      ++gap;
      // At the horizon the ego's front sits s0 + T v behind the lead's rear.
      const double s_rear = 60.0 - 2.25 + (6.0 * c.horizon_s);
      const auto k = static_cast<std::size_t>(std::lround(c.horizon_s / 0.1));
      EXPECT_NEAR(c.frenet[k].s + 3.9, s_rear - (2.0 + (1.5 * 6.0)), 1e-6);
      EXPECT_NEAR(c.frenet[k].s_dot, 6.0, 1e-6);
    }
  }
  EXPECT_EQ(gap, 15U * 3U);

  EgoObs ego;
  ego.pose = nuway_common::SE2{30.0, 0.5, 0.1};
  ego.vx_mps = 9.0;
  ego.ax_mps2 = 0.5;
  ego.steering_angle_rad = 0.05;
  const FrenetState f = EgoFrenetState(ego, route, LatticeOptions{}, 2.86, 25.0)
                            .value_or(FrenetState{});
  EXPECT_NEAR(f.s, 30.0, 1e-9);
  EXPECT_NEAR(f.d, 0.5, 1e-9);
  EXPECT_NEAR(f.s_dot, 9.0 * std::cos(0.1), 1e-9);
  EXPECT_NEAR(f.d_prime, std::tan(0.1), 1e-9);
  ego.pose.y = 12.0;
  EXPECT_FALSE(EgoFrenetState(ego, route, LatticeOptions{}, 2.86, std::nullopt)
                   .has_value());
}

}  // namespace
}  // namespace nuway_planning
