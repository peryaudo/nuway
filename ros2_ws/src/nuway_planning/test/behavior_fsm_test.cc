#include "nuway_planning/behavior_fsm.h"

#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <nuway_common/agents.h>
#include <nuway_common/frenet.h>
#include <nuway_common/geometry.h>
#include <nuway_map/lane_graph.h>
#include <nuway_map/opendrive_parser.h>

#include "nuway_planning/route_line.h"
#include "nuway_planning/scene.h"

namespace nuway_planning {
namespace {

using nuway_common::AgentClass;
using nuway_common::AgentState;
using nuway_common::kPi;
using nuway_common::PredictionSet;
using nuway_common::SE2;

// Two eastbound lanes on a 300 m straight road 1 (lane -1 at y = -1.75,
// lane -2 at y = -5), the mark between them broken both ways.
constexpr const char* kTwoLaneXodr = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="4" name="t"/>
  <road name="r1" length="300.0" id="1" junction="-1">
    <type s="0" type="town"><speed max="50" unit="km/h"/></type>
    <planView><geometry s="0" x="0" y="0" hdg="0" length="300"><line/></geometry></planView>
    <lanes><laneSection s="0">
      <center><lane id="0" type="none"><roadMark sOffset="0" type="solid" laneChange="none"/></lane></center>
      <right>
        <lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/><roadMark sOffset="0" type="broken" laneChange="both"/></lane>
        <lane id="-2" type="driving"><width sOffset="0" a="3.0" b="0" c="0" d="0"/><roadMark sOffset="0" type="solid" laneChange="none"/></lane>
      </right>
    </laneSection></lanes>
  </road>
</OpenDRIVE>)";

nuway_map::LaneGraph BuildGraph() {
  std::string error;
  const std::optional<nuway_map::OpenDriveMap> map =
      nuway_map::ParseOpenDrive(kTwoLaneXodr, &error);
  EXPECT_TRUE(map.has_value()) << error;
  return nuway_map::LaneGraph::Build(map.value_or(nuway_map::OpenDriveMap{}));
}

constexpr std::uint32_t kLaneOuter = (1U * 4096U) + (32U - 1U) + 1U;  // -1
constexpr std::uint32_t kLaneInner = (1U * 4096U) + (32U - 2U) + 1U;  // -2

// A route line along lane -1 (y = -1.75) from x = 0 to 250 (goal at 200).
RouteLine RouteOnLane(std::uint32_t lane_id, double y, double goal_s) {
  nuway_common::Vector2dList points;
  std::vector<std::uint32_t> lanes;
  std::vector<double> limits;
  for (int i = 0; i <= 500; ++i) {
    points.emplace_back(0.5 * i, y);
    lanes.push_back(lane_id);
    limits.push_back(13.9);
  }
  return RouteLine(nuway_common::ReferenceLine::FromPoints(points), lanes,
                   limits, {1.75}, {1.75}, goal_s);
}

// A route line along y = -1.75 whose lane ids switch from `first` to
// `second` at split_s: the ids stand in for two legs of a route that loops
// back through one junction.
RouteLine RouteWithLegs(std::uint32_t first, std::uint32_t second,
                        double split_s, double goal_s) {
  nuway_common::Vector2dList points;
  std::vector<std::uint32_t> lanes;
  std::vector<double> limits;
  for (int i = 0; i <= 500; ++i) {
    points.emplace_back(0.5 * i, -1.75);
    lanes.push_back(0.5 * i < split_s ? first : second);
    limits.push_back(13.9);
  }
  return RouteLine(nuway_common::ReferenceLine::FromPoints(points), lanes,
                   limits, {1.75}, {1.75}, goal_s);
}
// A route along y = -1.75 to x = 65 that then turns left on an R = 5 m arc
// (kappa 0.2): the bend past a stop line at x = 60.
RouteLine RouteWithBendPast65() {
  nuway_common::Vector2dList points;
  for (int i = 0; i <= 130; ++i) {
    points.emplace_back(0.5 * i, -1.75);
  }
  for (int i = 1; i <= 16; ++i) {
    const double phi = 0.1 * i;
    points.emplace_back(65.0 + (5.0 * std::sin(phi)),
                        3.25 - (5.0 * std::cos(phi)));
  }
  return RouteLine(nuway_common::ReferenceLine::FromPoints(points),
                   {kLaneOuter}, {13.9}, {1.75}, {1.75}, std::nullopt);
}

AgentState Car(std::uint32_t id, double x, double y, double yaw, double speed) {
  AgentState agent;
  agent.id = id;
  agent.class_id = AgentClass::kCar;
  agent.pose = SE2{x, y, yaw};
  agent.length_m = 4.5;
  agent.width_m = 2.0;
  agent.vx_mps = speed * std::cos(yaw);
  agent.vy_mps = speed * std::sin(yaw);
  return agent;
}

// Constant-velocity predictions of `agents`, T = 16 at 0.5 s.
PredictionSet Predict(const std::vector<AgentState>& agents) {
  PredictionSet set;
  set.num_samples = 1;
  set.num_timesteps = 16;
  set.dt_s = 0.5;
  set.sample_weight = {1.0};
  for (const AgentState& a : agents) {
    set.agent_ids.push_back(a.id);
    for (int t = 0; t < 16; ++t) {
      const double dt = 0.5 * (t + 1);
      set.xy.push_back(a.pose.x + (a.vx_mps * dt));
      set.xy.push_back(a.pose.y + (a.vy_mps * dt));
      set.yaw.push_back(a.pose.yaw);
    }
  }
  return set;
}

SceneInput Scene(const RouteLine* route, const nuway_map::LaneGraph* graph,
                 double x, double y, double v) {
  SceneInput in;
  in.ego.pose = SE2{x, y, 0.0};
  in.ego.vx_mps = v;
  in.route = route;
  in.graph = graph;
  in.predictions = Predict({});
  in.dt_s = 0.1;
  return in;
}

TEST(BehaviorFsmTest, FreeRoadKeepsTheLaneAtTheLimit) {
  const nuway_map::LaneGraph graph = BuildGraph();
  const RouteLine route = RouteOnLane(kLaneOuter, -1.75, 200.0);
  BehaviorFsm fsm{BehaviorFsmOptions{}};
  const BehaviorOutput out = fsm.Step(Scene(&route, &graph, 20.0, -1.75, 10.0));
  EXPECT_EQ(out.lateral, Lateral::kKeep);
  EXPECT_EQ(out.target_lane_id, kLaneOuter);
  EXPECT_EQ(out.longitudinal, Longitudinal::kFree);
  EXPECT_NEAR(out.target_speed_mps, 13.9, 1e-6);
  EXPECT_EQ(out.reason, "free");
  EXPECT_NEAR(out.ego_s, 20.0, 1e-9);
}

TEST(BehaviorFsmTest, NoRouteOrOffRouteIsTheNoInputDecision) {
  BehaviorFsm fsm{BehaviorFsmOptions{}};
  const BehaviorOutput none = fsm.Step(Scene(nullptr, nullptr, 0.0, 0.0, 0.0));
  EXPECT_EQ(none.longitudinal, Longitudinal::kStop);
  EXPECT_EQ(none.reason, "no_input");
  const RouteLine route = RouteOnLane(kLaneOuter, -1.75, 200.0);
  const BehaviorOutput off = fsm.Step(Scene(&route, nullptr, 20.0, 30.0, 5.0));
  EXPECT_EQ(off.longitudinal, Longitudinal::kStop);
  EXPECT_EQ(off.reason, "off_route");
}

TEST(BehaviorFsmTest, LeadVehicleAheadMeansFollowBelowItsSpeed) {
  const nuway_map::LaneGraph graph = BuildGraph();
  const RouteLine route = RouteOnLane(kLaneOuter, -1.75, 200.0);
  BehaviorFsm fsm{BehaviorFsmOptions{}};
  SceneInput in = Scene(&route, &graph, 20.0, -1.75, 12.0);
  in.agents = {
      Car(5, 40.0, -1.75, 0.0, 6.0),    // lead, 20 m ahead, slow
      Car(6, 40.0, -5.0, 0.0, 6.0),     // other lane: ignored
      Car(7, 5.0, -1.75, 0.0, 6.0),     // behind: ignored
      Car(8, 30.0, -1.75, M_PI, 0.0)};  // oncoming, standing: not a lead
  in.predictions = Predict(in.agents);
  const BehaviorOutput out = fsm.Step(in);
  EXPECT_EQ(out.longitudinal, Longitudinal::kFollow);
  EXPECT_EQ(out.lead_agent_id, 5U);
  EXPECT_LT(out.target_speed_mps, 12.0);  // IDM brakes at 2.9 s headway
  EXPECT_EQ(out.reason, "follow:agent5");
  // A lead 55 m ahead at the limit: IDM (s* = 22.9 m against a 49 m gap)
  // still eases off a little, as it does for any lead in range.
  in.agents = {Car(5, 75.0, -1.75, 0.0, 13.9)};
  in.predictions = Predict(in.agents);
  in.ego.vx_mps = 13.9;
  const BehaviorOutput far = fsm.Step(in);
  EXPECT_EQ(far.longitudinal, Longitudinal::kFollow);
  EXPECT_GT(far.target_speed_mps, 13.0);
  EXPECT_LT(far.target_speed_mps, 13.9);
}

TEST(BehaviorFsmTest, RedLightWithinStoppingDistanceMeansStop) {
  const nuway_map::LaneGraph graph = BuildGraph();
  const RouteLine route = RouteOnLane(kLaneOuter, -1.75, 200.0);
  BehaviorFsm fsm{BehaviorFsmOptions{}};
  TrafficLightObs light;
  light.id = 11;
  light.state = TrafficLightColor::kRed;
  light.stop_line = Eigen::Vector2d(60.0, -1.75);
  light.affected_lane_ids = {kLaneOuter};
  // 10 m/s: stopping distance 10^2 / 5 + 5 = 25 m. At 20 m the line is 40 m
  // away: still free.
  SceneInput in = Scene(&route, &graph, 20.0, -1.75, 10.0);
  in.lights = {light};
  EXPECT_EQ(fsm.Step(in).longitudinal, Longitudinal::kFree);
  // At 30 m the line is 29 m past stop_s: still free, but the target speed
  // is the one a 2.5 m/s^2 stop from there allows, so that STOP begins at
  // a speed its quintics can stop from (Decisions (g) of 2026-09-13).
  in.ego.pose.x = 30.0;
  const BehaviorOutput approach = fsm.Step(in);
  EXPECT_EQ(approach.longitudinal, Longitudinal::kFree);
  EXPECT_NEAR(approach.target_speed_mps, std::sqrt(2.0 * 2.5 * 29.0), 1e-6);
  in.ego.pose.x = 40.0;
  const BehaviorOutput out = fsm.Step(in);
  EXPECT_EQ(out.longitudinal, Longitudinal::kStop);
  EXPECT_NEAR(out.stop_s, 59.0, 1e-6);  // line - 1 m
  EXPECT_EQ(out.reason, "red_light");
  // Green: proceed. A light for another lane: ignored.
  in.lights[0].state = TrafficLightColor::kGreen;
  EXPECT_EQ(fsm.Step(in).longitudinal, Longitudinal::kFree);
  in.lights[0].state = TrafficLightColor::kRed;
  in.lights[0].affected_lane_ids = {kLaneInner};
  EXPECT_EQ(fsm.Step(in).longitudinal, Longitudinal::kFree);
  // Past the line, red or not: proceed (we are in the junction).
  in.lights[0].affected_lane_ids = {kLaneOuter};
  in.ego.pose.x = 62.0;
  EXPECT_EQ(fsm.Step(in).longitudinal, Longitudinal::kFree);
}

TEST(BehaviorFsmTest, AStopLineGovernsTheRouteOnlyWhereItProjects) {
  // dev03_01 (protocol v8): the route passed one junction westbound at
  // s = 33 and eastbound at s = 551, and the stop sign of the eastbound
  // lanes projected onto the westbound leg 3.5 m away; "its lane is
  // somewhere ahead" stopped the car there and marked the sign honoured
  // before the real pass. A governed lane must be the route lane within
  // stop_line_lane_window_m of the projected line; the window covers a line
  // short of the lanes it governs (the graph's sits on their boundary, a GT
  // light's up to 8 m before it), not a lane 40 m on.
  nuway_map::LaneGraph graph = BuildGraph();
  const RouteLine route = RouteWithLegs(kLaneInner, kLaneOuter, 100.0, 200.0);
  TrafficLightObs light;
  light.id = 11;
  light.state = TrafficLightColor::kRed;
  light.stop_line = Eigen::Vector2d(60.0, -1.75);
  light.affected_lane_ids = {kLaneOuter};
  {
    BehaviorFsm fsm{BehaviorFsmOptions{}};
    SceneInput in = Scene(&route, &graph, 40.0, -1.75, 10.0);
    in.lights = {light};
    EXPECT_EQ(fsm.Step(in).longitudinal, Longitudinal::kFree);
    in.lights[0].stop_line = Eigen::Vector2d(100.0, -1.75);
    in.ego.pose.x = 80.0;
    const BehaviorOutput out = fsm.Step(in);
    EXPECT_EQ(out.longitudinal, Longitudinal::kStop);
    EXPECT_EQ(out.reason, "red_light");
  }
  // The same rule for the lane graph's stop signs: boxes on lane -1 at
  // x = 60 (the later leg's lane, beside this leg) and at x = 100.
  nuway_map::StopSignSite beside;
  beside.center = Eigen::Vector2d(60.0, -1.75);
  beside.half_length_m = 1.7;
  beside.half_width_m = 1.7;
  nuway_map::StopSignSite boundary = beside;
  boundary.center = Eigen::Vector2d(100.0, -1.75);
  ASSERT_EQ(graph.AddStopSigns({beside, boundary}), 2U);
  BehaviorFsm fsm{BehaviorFsmOptions{}};
  SceneInput in = Scene(&route, &graph, 40.0, -1.75, 10.0);
  EXPECT_EQ(fsm.Step(in).longitudinal, Longitudinal::kFree);
  in.ego.pose.x = 80.0;
  const BehaviorOutput out = fsm.Step(in);
  EXPECT_EQ(out.longitudinal, Longitudinal::kStop);
  EXPECT_EQ(out.reason, "stop_sign");
  EXPECT_NEAR(out.stop_s, 99.0, 1e-6);
  // Halted 2.4 m short of stop_s (3.4 m from the line, where the sampler
  // holds it): the dwell counts from stop_s, so the sign is honoured after
  // stop_sign_hold_s and the car may go (ten 0.1 s ticks sum to just under
  // 1 s in floating point; the eleventh honours the sign).
  in.ego.pose.x = 96.6;
  in.ego.vx_mps = 0.0;
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(fsm.Step(in).longitudinal, Longitudinal::kStop);
  }
  EXPECT_EQ(fsm.Step(in).longitudinal, Longitudinal::kFree);
}

TEST(BehaviorFsmTest, YellowIsLatchedByTheDilemmaZoneRule) {
  const RouteLine route = RouteOnLane(kLaneOuter, -1.75, 200.0);
  BehaviorFsm fsm{BehaviorFsmOptions{}};
  TrafficLightObs light;
  light.id = 11;
  light.state = TrafficLightColor::kYellow;
  light.stop_line = Eigen::Vector2d(60.0, -1.75);
  light.affected_lane_ids = {kLaneOuter};
  light.yellow_duration_s = 3.0;
  light.time_in_state_s = 0.0;
  // 10 m/s, 22 m to the line: d_brake = 3 + 20 = 23 > 22 -> cannot stop
  // comfortably -> proceed, and the decision holds as the ego closes in.
  SceneInput in = Scene(&route, nullptr, 38.0, -1.75, 10.0);
  in.lights = {light};
  EXPECT_EQ(fsm.Step(in).longitudinal, Longitudinal::kFree);
  in.ego.pose.x = 50.0;
  EXPECT_EQ(fsm.Step(in).longitudinal, Longitudinal::kFree);
  // A fresh FSM at 8 m/s, 17 m out (inside the 17.8 m stop range): d_brake
  // 15.2 < 17, and 17 / 8 = 2.1 s does not beat the 1.5 s of yellow left
  // -> stop.
  BehaviorFsm fresh{BehaviorFsmOptions{}};
  in.ego.pose.x = 43.0;
  in.ego.vx_mps = 8.0;
  in.lights[0].time_in_state_s = 1.5;
  const BehaviorOutput out = fresh.Step(in);
  EXPECT_EQ(out.longitudinal, Longitudinal::kStop);
  EXPECT_EQ(out.reason, "yellow_light");
}

TEST(BehaviorFsmTest, YellowClearingUsesTheSpeedTheBendAheadAllows) {
  TrafficLightObs light;
  light.id = 11;
  light.state = TrafficLightColor::kYellow;
  light.stop_line = Eigen::Vector2d(60.0, -1.75);
  light.affected_lane_ids = {kLaneOuter};
  light.yellow_duration_s = 3.0;
  light.time_in_state_s = 0.0;
  // 5 m/s, 11.6 m out: d_brake = 1.5 + 5 = 6.5 < 11.6, and at 5 m/s the
  // line is cleared in 2.3 s of the 3 s yellow: proceed on a straight.
  const RouteLine straight = RouteOnLane(kLaneOuter, -1.75, 200.0);
  BehaviorFsm on_straight{BehaviorFsmOptions{}};
  SceneInput in = Scene(&straight, nullptr, 48.4, -1.75, 5.0);
  in.lights = {light};
  EXPECT_EQ(on_straight.Step(in).longitudinal, Longitudinal::kFree);
  // The same approach with an R = 5 m turn past the line: the sampler drives
  // it at sqrt(2 / 0.2) = 3.2 m/s, which takes 3.7 s to the line: the
  // latched decision is a stop. STOP itself begins one stopping distance
  // plus stop_lookahead_m (10 m) out, so the first tick is FREE with the
  // approach bound and the next, 9 m out, is the stop.
  const RouteLine bend = RouteWithBendPast65();
  EXPECT_NEAR(bend.CurvatureSpeedCap(48.4, 60.0, 2.0), 3.16, 0.3);
  BehaviorFsm on_bend{BehaviorFsmOptions{}};
  in.route = &bend;
  EXPECT_EQ(on_bend.Step(in).longitudinal, Longitudinal::kFree);
  in.ego.pose.x = 51.0;
  const BehaviorOutput out = on_bend.Step(in);
  EXPECT_EQ(out.longitudinal, Longitudinal::kStop);
  EXPECT_EQ(out.reason, "yellow_light");
  // And a straight from 9 m out proceeds: the latch is per approach.
  BehaviorFsm late{BehaviorFsmOptions{}};
  in.route = &straight;
  EXPECT_EQ(late.Step(in).longitudinal, Longitudinal::kFree);
}

TEST(BehaviorFsmTest, CrossingAgentAheadMeansYield) {
  const RouteLine route = RouteOnLane(kLaneOuter, -1.75, 200.0);
  BehaviorFsm fsm{BehaviorFsmOptions{}};
  SceneInput in = Scene(&route, nullptr, 20.0, -1.75, 8.0);
  // A car 15 m to the left of the route at x = 50, heading south at 6 m/s:
  // it crosses the ego lane (y = -1.75) after ~2.2 s; the ego needs 3.75 s
  // to get there, so the car arrives first: yield.
  in.agents = {Car(9, 50.0, 12.0, -kPi / 2.0, 6.0)};
  in.predictions = Predict(in.agents);
  const BehaviorOutput out = fsm.Step(in);
  EXPECT_EQ(out.longitudinal, Longitudinal::kYield);
  // The car's body reaches the route at x ~ 49 (half its 2 m width before
  // its centre line at 50); the rear axle stops 3 m plus the 3.9 m bumper
  // short of that, so the nose stands 3 m from the crossing body.
  const BehaviorFsmOptions defaults;
  EXPECT_NEAR(out.stop_s, 49.0 - 3.0 - defaults.ego_front_m, 0.6);
  EXPECT_EQ(out.reason, "yield:agent9");
  // The same car far ahead in time (already past the lane at t = 0.5 s
  // after starting below the lane): no conflict, but the yield is held for
  // yield_hold_s at the same stop before the decision returns to free.
  in.agents = {Car(9, 50.0, -8.0, -kPi / 2.0, 6.0)};
  in.predictions = Predict(in.agents);
  const BehaviorOutput held = fsm.Step(in);
  EXPECT_EQ(held.longitudinal, Longitudinal::kYield);
  EXPECT_NEAR(held.stop_s, out.stop_s, 1e-9);
  EXPECT_EQ(held.reason, "yield:agent9(held)");
  for (int i = 0; i < 8; ++i) {  // dt 0.1 s: ages 0.2 .. 0.9 s
    EXPECT_EQ(fsm.Step(in).longitudinal, Longitudinal::kYield);
  }
  EXPECT_EQ(fsm.Step(in).longitudinal, Longitudinal::kFree);
  // A fresh machine sees no conflict at all.
  BehaviorFsm fresh{BehaviorFsmOptions{}};
  EXPECT_EQ(fresh.Step(in).longitudinal, Longitudinal::kFree);
}

TEST(BehaviorFsmTest, RouteOnTheNeighbourLaneRequestsAChange) {
  const nuway_map::LaneGraph graph = BuildGraph();
  // The route runs on lane -2 (y = -5) while the ego sits on lane -1.
  const RouteLine route = RouteOnLane(kLaneInner, -5.0, 200.0);
  BehaviorFsm fsm{BehaviorFsmOptions{}};
  SceneInput in = Scene(&route, &graph, 20.0, -1.75, 8.0);
  BehaviorOutput out = fsm.Step(in);
  EXPECT_EQ(out.lateral, Lateral::kChangeRight);
  EXPECT_EQ(out.target_lane_id, kLaneInner);
  // The change is held while the ego crosses over, and released once it
  // is on the target lane and the commit time has passed.
  in.ego.pose.y = -3.4;
  for (int i = 0; i < 5; ++i) {
    out = fsm.Step(in);
    EXPECT_EQ(out.lateral, Lateral::kChangeRight);
  }
  in.ego.pose.y = -5.0;
  for (int i = 0; i < 40; ++i) {
    out = fsm.Step(in);
  }
  EXPECT_EQ(out.lateral, Lateral::kKeep);
  EXPECT_EQ(out.target_lane_id, kLaneInner);
}

TEST(BehaviorFsmTest, GoalWithinStoppingDistanceMeansStop) {
  const RouteLine route = RouteOnLane(kLaneOuter, -1.75, 100.0);
  BehaviorFsm fsm{BehaviorFsmOptions{}};
  const BehaviorOutput far = fsm.Step(Scene(&route, nullptr, 20.0, -1.75, 8.0));
  EXPECT_EQ(far.longitudinal, Longitudinal::kFree);
  const BehaviorOutput out = fsm.Step(Scene(&route, nullptr, 85.0, -1.75, 8.0));
  EXPECT_EQ(out.longitudinal, Longitudinal::kStop);
  EXPECT_NEAR(out.stop_s, 99.0, 1e-6);
  EXPECT_EQ(out.reason, "goal");
}

TEST(BehaviorFsmTest, AStandingAgentIsNotAYield) {
  // A walker standing on the corridor 25 m ahead is not crossing anything:
  // yielding to it held the car forever on a protocol route. It is the
  // collision check's, which stops the car before it if it stays.
  const RouteLine route = RouteOnLane(kLaneOuter, -1.75, 200.0);
  BehaviorFsm fsm{BehaviorFsmOptions{}};
  SceneInput in = Scene(&route, nullptr, 20.0, -1.75, 8.0);
  AgentState walker = Car(7, 45.0, -1.0, -kPi / 2.0, 0.0);
  walker.class_id = nuway_common::AgentClass::kPedestrian;
  walker.length_m = 0.4;
  walker.width_m = 0.4;
  in.agents = {walker};
  in.predictions = Predict(in.agents);
  // It stands in the lane, so FOLLOW takes it as a stopped lead (the
  // gap-keeping candidate stops behind it); what matters is no YIELD.
  const BehaviorOutput standing = fsm.Step(in);
  EXPECT_NE(standing.longitudinal, Longitudinal::kYield);
  EXPECT_EQ(standing.longitudinal, Longitudinal::kFollow);
  // The same walker stepping across at 1 m/s is a yield.
  in.agents[0].vy_mps = -1.0;
  in.predictions = Predict(in.agents);
  BehaviorFsm fresh{BehaviorFsmOptions{}};
  EXPECT_EQ(fresh.Step(in).longitudinal, Longitudinal::kYield);
}

}  // namespace
}  // namespace nuway_planning
