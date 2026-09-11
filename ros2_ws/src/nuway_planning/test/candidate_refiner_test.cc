#include "nuway_planning/candidate_refiner.h"

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <nuway_common/agents.h>
#include <nuway_common/frenet.h>
#include <nuway_common/geometry.h>

#include "nuway_planning/behavior_fsm.h"
#include "nuway_planning/candidate.h"
#include "nuway_planning/collision_checker.h"
#include "nuway_planning/lattice_sampler.h"
#include "nuway_planning/route_line.h"
#include "nuway_planning/scene.h"

namespace nuway_planning {
namespace {

using nuway_common::AgentState;
using nuway_common::FrenetState;

// A 300 m straight with 3.5 m of drivable width on each side.
RouteLine Straight() {
  nuway_common::Vector2dList points;
  for (int i = 0; i <= 600; ++i) {
    points.emplace_back(0.5 * i, 0.0);
  }
  return RouteLine(nuway_common::ReferenceLine::FromPoints(points), {1U},
                   {13.9}, {3.5}, {3.5}, 250.0);
}

FrenetState Ego(double s, double v) {
  FrenetState f;
  f.s = s;
  f.s_dot = v;
  return f;
}

// The centre-line, full-horizon velocity-keeping candidate at v_target.
Candidate CentreCandidate(const SceneInput& in, const FrenetState& ego,
                          double v_target) {
  BehaviorOutput decision;
  decision.longitudinal = Longitudinal::kFree;
  decision.target_speed_mps = v_target;
  const LatticeSampler sampler{LatticeOptions{}, LatticeLimits{}};
  for (const Candidate& c : sampler.Sample(in, ego, decision)) {
    if (!c.injected && std::abs(c.d_f_m) < 1e-9 && c.horizon_s == 8.0 &&
        std::abs(c.v_target_mps - v_target) < 1e-9 && c.s_f_m > 60.0) {
      return c;
    }
  }
  return Candidate{};
}

AgentState Car(std::uint32_t id, double x, double y, double vx) {
  AgentState a;
  a.id = id;
  a.class_id = nuway_common::AgentClass::kCar;
  a.pose = nuway_common::SE2{x, y, 0.0};
  a.length_m = 4.5;
  a.width_m = 2.0;
  a.vx_mps = vx;
  return a;
}

nuway_common::PredictionSet ConstVel(const std::vector<AgentState>& agents) {
  nuway_common::PredictionSet set;
  set.num_samples = 1;
  set.num_timesteps = 16;
  set.dt_s = 0.5;
  set.sample_weight = {1.0};
  for (const AgentState& a : agents) {
    set.agent_ids.push_back(a.id);
    for (int t = 0; t < 16; ++t) {
      set.xy.push_back(a.pose.x + (a.vx_mps * 0.5 * (t + 1)));
      set.xy.push_back(a.pose.y);
      set.yaw.push_back(0.0);
    }
  }
  return set;
}

TEST(CandidateRefinerTest, ParkedCarOnTheRightShiftsThePathLeft) {
  const RouteLine route = Straight();
  SceneInput in;
  in.route = &route;
  in.agents = {Car(3, 60.0, -1.2, 0.0)};
  in.predictions = ConstVel(in.agents);
  const FrenetState ego = Ego(20.0, 10.0);
  Candidate c = CentreCandidate(in, ego, 10.0);
  ASSERT_EQ(c.trajectory.size(), 81U);
  CandidateRefiner refiner{RefinerOptions{}, LatticeLimits{},
                           CollisionOptions{}};
  const RefineOutcome out = refiner.Refine(in, ego, &c);
  EXPECT_EQ(out.path, QpOutcome::kSolved) << out.message;
  EXPECT_EQ(out.speed, QpOutcome::kSolved) << out.message;
  EXPECT_TRUE(out.refined());
  EXPECT_TRUE(c.refined);
  EXPECT_FALSE(c.qp_relaxed);
  ASSERT_EQ(c.frenet.size(), 81U);
  // Clearance: -1.2 + 1.0 + 0.2 + 0.92 + 0.4 = 1.32 m alongside the car.
  bool passed = false;
  for (const FrenetState& f : c.frenet) {
    if (f.s > 58.0 && f.s < 62.0) {
      EXPECT_GE(f.d, 1.32 - 1e-2);
      passed = true;
    }
    EXPECT_LE(std::abs(f.d), 3.5 - 0.92 + 1e-2);
  }
  EXPECT_TRUE(passed);
  EXPECT_NEAR(c.frenet.front().d, 0.0, 1e-3);
  EXPECT_LT(std::abs(c.frenet.back().d), 0.5);  // back toward the centre
  // The map-frame samples follow the refined offset.
  EXPECT_GT(c.trajectory[45].y, 1.0);
}

TEST(CandidateRefinerTest, SlowLeadBoundsTheSpeedProfile) {
  const RouteLine route = Straight();
  SceneInput in;
  in.route = &route;
  in.agents = {Car(4, 45.0, 0.0, 4.0)};
  in.predictions = ConstVel(in.agents);
  const FrenetState ego = Ego(20.0, 10.0);
  Candidate c = CentreCandidate(in, ego, 10.0);
  ASSERT_EQ(c.trajectory.size(), 81U);
  CandidateRefiner refiner{RefinerOptions{}, LatticeLimits{},
                           CollisionOptions{}};
  refiner.OnPlanningTick();  // no solution yet: a no-op
  const RefineOutcome out = refiner.Refine(in, ego, &c);
  EXPECT_TRUE(out.refined()) << out.message;
  EXPECT_FALSE(c.qp_relaxed) << out.message;
  EXPECT_LT(out.speed_iterations, 2000);
  // Behind the lead's rear minus the margins: 45 - 2.25 - 0.2 - 3.9 - 1.0
  // + 4 t. The lattice profile at 10 m/s would have crossed it at 3.3 s.
  for (std::size_t i = 0; i < c.frenet.size(); ++i) {
    const double t = c.trajectory[i].t;
    EXPECT_LE(c.frenet[i].s, 45.0 - 7.35 + (4.0 * t) + 1e-2) << t;
    EXPECT_GE(c.trajectory[i].v, -1e-3);
    EXPECT_GE(c.trajectory[i].a, -6.0 - 1e-2);
    EXPECT_LE(c.trajectory[i].a, 3.0 + 1e-2);
  }
  EXPECT_LT(c.trajectory.back().v, 6.0);
  EXPECT_GT(c.trajectory.back().v, 2.0);
  EXPECT_GT(c.frenet.back().s, 50.0);
}

TEST(CandidateRefinerTest, LeadInsideTheMarginClosesTheBoxWithoutASolve) {
  // A lead 5 m ahead moving at 4 m/s: its inflated rear (5 - 7.35 m) is
  // behind the ego, so no speed profile can stay behind it. The refiner
  // reports the closed box instead of burning the QP budget.
  const RouteLine route = Straight();
  SceneInput in;
  in.route = &route;
  in.agents = {Car(4, 25.0, 0.0, 4.0)};
  in.predictions = ConstVel(in.agents);
  const FrenetState ego = Ego(20.0, 10.0);
  Candidate c = CentreCandidate(in, ego, 10.0);
  CandidateRefiner refiner{RefinerOptions{}, LatticeLimits{},
                           CollisionOptions{}};
  const RefineOutcome out = refiner.Refine(in, ego, &c);
  EXPECT_FALSE(out.refined());
  EXPECT_EQ(out.message.rfind("speed box closed", 0), 0U) << out.message;
  EXPECT_EQ(out.speed_iterations, 0);
  EXPECT_TRUE(c.qp_relaxed);
}

TEST(CandidateRefinerTest, ExhaustedBudgetKeepsTheLatticeShape) {
  const RouteLine route = Straight();
  SceneInput in;
  in.route = &route;
  const FrenetState ego = Ego(20.0, 10.0);
  Candidate c = CentreCandidate(in, ego, 10.0);
  const Candidate before = c;
  RefinerOptions options;
  options.qp.max_iter = 1;
  options.qp.check_termination = 1;
  CandidateRefiner refiner{options, LatticeLimits{}, CollisionOptions{}};
  const RefineOutcome out = refiner.Refine(in, ego, &c);
  EXPECT_FALSE(out.refined());
  // (The path QP's zero start is already optimal for a centre-line
  // candidate, so it may converge in that one iteration; the speed QP
  // cannot.)
  EXPECT_EQ(out.speed, QpOutcome::kFailed);
  EXPECT_NE(out.message.find("max_iter_reached"), std::string::npos)
      << out.message;
  EXPECT_FALSE(c.refined);
  EXPECT_TRUE(c.qp_relaxed);
  ASSERT_EQ(c.trajectory.size(), before.trajectory.size());
  for (std::size_t i = 0; i < c.trajectory.size(); ++i) {
    EXPECT_EQ(c.trajectory[i].x, before.trajectory[i].x);
    EXPECT_EQ(c.frenet[i].s, before.frenet[i].s);
  }
  // A short candidate near the line end skips the path QP.
  const FrenetState late = Ego(298.0, 1.0);
  Candidate hold = CentreCandidate(in, late, 1.0);
  if (hold.trajectory.empty()) {
    BehaviorOutput decision;
    decision.longitudinal = Longitudinal::kFree;
    decision.target_speed_mps = 1.0;
    const LatticeSampler sampler{LatticeOptions{}, LatticeLimits{}};
    hold = sampler.Sample(in, late, decision).front();
  }
  CandidateRefiner fine{RefinerOptions{}, LatticeLimits{}, CollisionOptions{}};
  const RefineOutcome skipped = fine.Refine(in, late, &hold);
  EXPECT_TRUE(skipped.path_skipped);
  EXPECT_TRUE(skipped.refined()) << skipped.message;
  fine.Reset();
}

}  // namespace
}  // namespace nuway_planning
