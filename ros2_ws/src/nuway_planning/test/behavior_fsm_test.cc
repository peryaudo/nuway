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
  in.agents = {Car(5, 40.0, -1.75, 0.0, 6.0),  // lead, 20 m ahead, slow
               Car(6, 40.0, -5.0, 0.0, 6.0),   // other lane: ignored
               Car(7, 5.0, -1.75, 0.0, 6.0)};  // behind: ignored
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
  EXPECT_NEAR(out.stop_s, 47.0, 0.6);  // conflict at ~50, minus 3
  EXPECT_EQ(out.reason, "yield:agent9");
  // The same car far ahead in time (already past the lane at t = 0.5 s
  // after starting below the lane): no conflict.
  in.agents = {Car(9, 50.0, -8.0, -kPi / 2.0, 6.0)};
  in.predictions = Predict(in.agents);
  EXPECT_EQ(fsm.Step(in).longitudinal, Longitudinal::kFree);
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

}  // namespace
}  // namespace nuway_planning
