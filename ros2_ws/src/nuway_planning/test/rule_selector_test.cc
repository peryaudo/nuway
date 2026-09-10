#include "nuway_planning/rule_selector.h"

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <nuway_common/agents.h>
#include <nuway_common/geometry.h>

#include "nuway_planning/behavior_fsm.h"
#include "nuway_planning/candidate.h"
#include "nuway_planning/planner.h"
#include "nuway_planning/route_line.h"
#include "nuway_planning/scene.h"

namespace nuway_planning {
namespace {

using nuway_common::AgentState;

RouteLine Straight() {
  nuway_common::Vector2dList points;
  for (int i = 0; i <= 600; ++i) {
    points.emplace_back(0.5 * i, 0.0);
  }
  return RouteLine(nuway_common::ReferenceLine::FromPoints(points), {1U},
                   {13.9}, {1.75}, {1.75}, 250.0);
}

AgentState ParkedCar(std::uint32_t id, double x, double y) {
  AgentState a;
  a.id = id;
  a.class_id = nuway_common::AgentClass::kCar;
  a.pose = nuway_common::SE2{x, y, 0.0};
  a.length_m = 4.5;
  a.width_m = 2.0;
  return a;
}

SceneInput Scene(const RouteLine* route, double x, double v) {
  SceneInput in;
  in.route = route;
  in.ego.pose = nuway_common::SE2{x, 0.0, 0.0};
  in.ego.vx_mps = v;
  return in;
}

BehaviorOutput Free(double v) {
  BehaviorOutput d;
  d.longitudinal = Longitudinal::kFree;
  d.target_speed_mps = v;
  d.reason = "free";
  return d;
}

// Every selected candidate is refined or injected, feasible, and the
// cheapest among those (M1 §3.6, task 7).
void ExpectValidSelection(const PlanResult& r) {
  ASSERT_GE(r.selected_index, 0);
  const Candidate& s = r.candidates[static_cast<std::size_t>(r.selected_index)];
  EXPECT_TRUE(s.refined || s.injected);
  EXPECT_TRUE(s.feasible());
  for (const Candidate& c : r.candidates) {
    if ((c.refined || c.injected) && c.feasible()) {
      EXPECT_LE(s.cost, c.cost);
    }
  }
  EXPECT_LE(r.candidates.size(), 32U);
}

TEST(RuleSelectorTest, FeasibleNormalCandidateBeatsTheStopPair) {
  const RouteLine route = Straight();
  Planner planner{PlannerOptions{}};
  const PlanResult r = planner.Plan(Scene(&route, 20.0, 10.0), Free(10.0));
  EXPECT_FALSE(r.no_input);
  EXPECT_TRUE(r.used_decision);
  EXPECT_EQ(r.sampled, 182);
  EXPECT_GT(r.feasible, 50);
  EXPECT_EQ(r.refined, 8);
  ExpectValidSelection(r);
  const Candidate* s = r.selected();
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->source, "lattice");
  EXPECT_FALSE(s->injected);
  EXPECT_TRUE(s->refined);
  EXPECT_GT(s->trajectory.back().v, 5.0);  // it drives on
  // The published list: refined and injected first, sorted by cost.
  int leading = 0;
  for (const Candidate& c : r.candidates) {
    if (!(c.refined || c.injected)) {
      break;
    }
    ++leading;
  }
  EXPECT_EQ(leading, 10);
  for (std::size_t i = 1; i < 10; ++i) {
    EXPECT_LE(r.candidates[i - 1].cost, r.candidates[i].cost);
  }
  EXPECT_EQ(r.candidates.front().cost_breakdown.size(), kNumCostTerms);
  // A second tick: the consistency term sees the previous choice and the
  // selection stays a driving candidate.
  const PlanResult again = planner.Plan(Scene(&route, 22.0, 10.0), Free(10.0));
  ExpectValidSelection(again);
  EXPECT_EQ(again.selected()->source, "lattice");
}

TEST(RuleSelectorTest, BlockedLaneLeavesTheGentleStopThenTheHardOne) {
  const RouteLine route = Straight();
  // A car across the lane 60 m ahead at 10 m/s: every keeping candidate
  // (at least 7 m/s for 8 s: the front reaches 81 m against the car's
  // inflated rear at 77.5 m) hits it, the gentle stop (rests at 53 m,
  // front at 58 m) clears it.
  SceneInput in = Scene(&route, 20.0, 10.0);
  in.agents = {ParkedCar(1, 80.0, 0.0)};
  Planner planner{PlannerOptions{}};
  const PlanResult gentle = planner.Plan(in, Free(10.0));
  EXPECT_EQ(gentle.feasible, 0);
  ExpectValidSelection(gentle);
  EXPECT_EQ(gentle.selected()->source, "stop");
  EXPECT_EQ(gentle.selected()->speed_kind, SpeedKind::kGentleStop);
  EXPECT_EQ(gentle.candidates.size(), 2U);
  // 40 m ahead: the gentle stop's front (58 m) passes the car's inflated
  // rear (57.5 m), the hard one (rests at 28 m, front at 33 m) does not.
  in.agents = {ParkedCar(1, 60.0, 0.0)};
  Planner other{PlannerOptions{}};
  const PlanResult hard = other.Plan(in, Free(10.0));
  ExpectValidSelection(hard);
  EXPECT_EQ(hard.selected()->speed_kind, SpeedKind::kHardStop);
  EXPECT_GT(hard.candidates[1].cost, hard.candidates[0].cost);
}

TEST(RuleSelectorTest, NoDecisionPlansTheStopPairAlone) {
  const RouteLine route = Straight();
  Planner planner{PlannerOptions{}};
  const PlanResult none = planner.Plan(Scene(&route, 20.0, 10.0), std::nullopt);
  EXPECT_FALSE(none.no_input);
  EXPECT_FALSE(none.used_decision);
  EXPECT_EQ(none.sampled, 2);
  ExpectValidSelection(none);
  EXPECT_EQ(none.selected()->source, "stop");
  EXPECT_EQ(none.selected()->speed_kind, SpeedKind::kGentleStop);
  // The no-input decision counts as no decision.
  const PlanResult no_input =
      planner.Plan(Scene(&route, 22.0, 10.0), NoInputDecision());
  EXPECT_FALSE(no_input.used_decision);
  EXPECT_EQ(no_input.selected()->speed_kind, SpeedKind::kGentleStop);
  // Off the line, or without one: the no-input case (the node's "none").
  const PlanResult off = planner.Plan(Scene(&route, 22.0, 10.0), Free(10.0));
  EXPECT_FALSE(off.no_input);
  SceneInput far = Scene(&route, 22.0, 10.0);
  far.ego.pose.y = 30.0;
  EXPECT_TRUE(planner.Plan(far, Free(10.0)).no_input);
  EXPECT_TRUE(planner.Plan(Scene(nullptr, 22.0, 10.0), Free(10.0)).no_input);
  planner.Reset();
}

TEST(RuleSelectorTest, SelectIgnoresUnrefinedNormals) {
  std::vector<Candidate> set(3);
  set[0].cost = 1.0;  // unrefined normal: never selectable
  set[1].cost = 5.0;
  set[1].refined = true;
  set[2].cost = 3.0;
  set[2].injected = true;
  set[2].reject = "collision";  // infeasible injected: not selectable either
  EXPECT_EQ(RuleSelector::Select(set), 1);
  set[2].reject.clear();
  EXPECT_EQ(RuleSelector::Select(set), 2);
  EXPECT_EQ(RuleSelector::Select({}), -1);
  EXPECT_STREQ(kCostTermNames[0], "collision");
  EXPECT_STREQ(kCostTermNames[9], "qp_relaxed");
}

}  // namespace
}  // namespace nuway_planning
