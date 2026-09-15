#include "nuway_prediction/const_vel.h"

#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <nuway_common/agents.h>
#include <nuway_common/geometry.h>
#include <nuway_map/lane_graph.h>
#include <nuway_map/opendrive_parser.h>

namespace nuway_prediction {
namespace {

using nuway_common::AgentClass;
using nuway_common::AgentState;
using nuway_common::kPi;
using nuway_common::PredictionSet;
using nuway_common::SE2;

// A closed circle of radius 30 m centred at the origin (one counter-clockwise
// arc): lane -1 lies outside at radius 31.75 driving counter-clockwise, lane
// 1 inside at 28.25 driving clockwise; each is its own successor.
constexpr const char* kCircleXodr = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="4" name="c"/>
  <road name="ring" length="188.49555921538757" id="7" junction="-1">
    <link>
      <predecessor elementType="road" elementId="7" contactPoint="end"/>
      <successor elementType="road" elementId="7" contactPoint="start"/>
    </link>
    <planView><geometry s="0" x="30" y="0" hdg="1.5707963267948966" length="188.49555921538757"><arc curvature="0.03333333333333333"/></geometry></planView>
    <lanes><laneSection s="0">
      <left><lane id="1" type="driving"><link><predecessor id="1"/><successor id="1"/></link><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></left>
      <center><lane id="0" type="none"/></center>
      <right><lane id="-1" type="driving"><link><predecessor id="-1"/><successor id="-1"/></link><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></right>
    </laneSection></lanes>
  </road>
</OpenDRIVE>)";

nuway_map::LaneGraph BuildGraph(const char* xml) {
  std::string error;
  const std::optional<nuway_map::OpenDriveMap> map =
      nuway_map::ParseOpenDrive(xml, &error);
  EXPECT_TRUE(map.has_value()) << error;
  return nuway_map::LaneGraph::Build(map.value_or(nuway_map::OpenDriveMap{}));
}

AgentState Car(double x, double y, double yaw, double speed) {
  AgentState agent;
  agent.id = 42;
  agent.class_id = AgentClass::kCar;
  agent.pose = SE2{x, y, yaw};
  agent.length_m = 4.5;
  agent.width_m = 2.0;
  agent.vx_mps = speed * std::cos(yaw);
  agent.vy_mps = speed * std::sin(yaw);
  return agent;
}

TEST(ConstVelTest, StraightLineWithoutLaneGraph) {
  const ConstVelOptions options;
  const ConstVelPredictor predictor(options);
  const AgentState car = Car(1.0, 2.0, 0.5, 10.0);
  const AgentFuture future = predictor.PredictAgent(car);
  ASSERT_EQ(future.poses.size(), 16U);
  for (int t = 0; t < 16; ++t) {
    const double dt = 0.5 * (t + 1);
    const SE2& p = future.poses[static_cast<std::size_t>(t)];
    EXPECT_NEAR(p.x, 1.0 + (10.0 * std::cos(0.5) * dt), 1e-9);
    EXPECT_NEAR(p.y, 2.0 + (10.0 * std::sin(0.5) * dt), 1e-9);
    EXPECT_NEAR(p.yaw, 0.5, 1e-12);
  }
}

TEST(ConstVelTest, LaneFollowStaysOnACurvedLane) {
  const nuway_map::LaneGraph graph = BuildGraph(kCircleXodr);
  const ConstVelOptions options;
  ConstVelPredictor predictor(options);
  predictor.set_lane_graph(&graph);
  // On the outer lane (radius 31.75) at angle 0, heading +y (counter-
  // clockwise), 8 m/s: 64 m in 8 s, i.e. a 2 rad sweep, far beyond where
  // the straight line leaves the road.
  const double r = 31.75;
  const AgentState car = Car(r, 0.0, kPi / 2.0, 8.0);
  const AgentFuture future = predictor.PredictAgent(car);
  ASSERT_EQ(future.poses.size(), 16U);
  for (int t = 0; t < 16; ++t) {
    const SE2& p = future.poses[static_cast<std::size_t>(t)];
    // The centerline is a 1 m polyline of the arc, so the predicted radius
    // is within the chord sagitta (< 5 mm) of the lane radius.
    EXPECT_NEAR(std::hypot(p.x, p.y), r, 0.02) << "t=" << t;
    const double angle = std::atan2(p.y, p.x);
    const double expected_angle = (8.0 * 0.5 * (t + 1)) / r;
    EXPECT_NEAR(nuway_common::WrapAngle(angle - expected_angle), 0.0, 0.02)
        << "t=" << t;
    EXPECT_NEAR(nuway_common::WrapAngle(p.yaw - (angle + (kPi / 2.0))), 0.0,
                0.05)
        << "t=" << t;
  }
  // The straight line would have left the circle by 8 s.
  const SE2 last = future.poses.back();
  EXPECT_LT(std::hypot(last.x, last.y), 33.0);
}

TEST(ConstVelTest, LaneFollowKeepsTheLateralOffset) {
  const nuway_map::LaneGraph graph = BuildGraph(kCircleXodr);
  const ConstVelOptions options;
  ConstVelPredictor predictor(options);
  predictor.set_lane_graph(&graph);
  // 0.5 m outside the lane centre (radius 32.25), still on the lane.
  const AgentState car = Car(32.25, 0.0, kPi / 2.0, 5.0);
  const AgentFuture future = predictor.PredictAgent(car);
  for (const SE2& p : future.poses) {
    EXPECT_NEAR(std::hypot(p.x, p.y), 32.25, 0.03);
  }
}

TEST(ConstVelTest, LaneFollowFallsBackToStraightAgainstTheLane) {
  const nuway_map::LaneGraph graph = BuildGraph(kCircleXodr);
  const ConstVelOptions options;
  ConstVelPredictor predictor(options);
  predictor.set_lane_graph(&graph);
  // Heading matches the outer lane but the velocity points backwards.
  AgentState car = Car(31.75, 0.0, kPi / 2.0, 5.0);
  car.vx_mps = 0.0;
  car.vy_mps = -5.0;
  const AgentFuture future = predictor.PredictAgent(car);
  EXPECT_NEAR(future.poses[0].x, 31.75, 1e-9);
  EXPECT_NEAR(future.poses[0].y, -2.5, 1e-9);
}

TEST(ConstVelTest, PedestriansAreClampedAndStaticsStayPut) {
  const ConstVelOptions options;
  const ConstVelPredictor predictor(options);
  AgentState walker = Car(0.0, 0.0, 0.0, 6.0);
  walker.class_id = AgentClass::kPedestrian;
  const AgentFuture w = predictor.PredictAgent(walker);
  EXPECT_NEAR(w.poses[0].x, 1.0, 1e-9);  // 2 m/s * 0.5 s
  AgentState prop = Car(3.0, 4.0, 0.0, 6.0);
  prop.class_id = AgentClass::kStaticObstacle;
  const AgentFuture p = predictor.PredictAgent(prop);
  EXPECT_NEAR(p.poses.back().x, 3.0, 1e-12);
  EXPECT_NEAR(p.poses.back().y, 4.0, 1e-12);
}

TEST(ConstVelTest, PredictPacksOneJointSample) {
  ConstVelOptions options;
  options.num_timesteps = 3;
  const ConstVelPredictor predictor(options);
  std::vector<AgentState> agents = {Car(0.0, 0.0, 0.0, 2.0),
                                    Car(10.0, 0.0, kPi, 2.0)};
  agents[1].id = 43;
  const PredictionSet set = predictor.Predict(agents);
  EXPECT_EQ(set.num_samples, 1);
  EXPECT_EQ(set.num_timesteps, 3);
  ASSERT_EQ(set.num_agents(), 2);
  ASSERT_EQ(set.xy.size(), 12U);
  ASSERT_EQ(set.yaw.size(), 6U);
  EXPECT_NEAR(set.PoseAt(0, 0, 2).x, 3.0, 1e-9);
  EXPECT_NEAR(set.PoseAt(0, 1, 2).x, 7.0, 1e-9);
  EXPECT_NEAR(set.TimeAt(2), 1.5, 1e-12);
  EXPECT_EQ(set.IndexOf(43), 1);
}

}  // namespace
}  // namespace nuway_prediction
