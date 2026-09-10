// The plain-struct inputs every M1 planning library consumes (M1 §3.2-§3.7):
// the ego observation, traffic lights, and the SceneInput bundle a node
// assembles per planning tick from the messages (msg_conv.h). Everything is
// in the map frame after the agent frame rule of M1 §3.1. No rclcpp.
#ifndef NUWAY_PLANNING_SCENE_H_
#define NUWAY_PLANNING_SCENE_H_

#include <cstdint>
#include <vector>

#include <nuway_common/agents.h>
#include <nuway_common/geometry.h>
#include <nuway_map/lane_graph.h>
#include <nuway_msgs/msg/traffic_light.hpp>

#include "nuway_planning/route_line.h"

namespace nuway_planning {

// EgoState's planning-relevant fields: base_link (rear axle) pose in map,
// body-frame velocity and acceleration, front wheel angle.
struct EgoObs {
  nuway_common::SE2 pose;
  double vx_mps = 0.0;
  double vy_mps = 0.0;
  double ax_mps2 = 0.0;
  double ay_mps2 = 0.0;
  double yaw_rate_radps = 0.0;
  double steering_angle_rad = 0.0;
};

// TrafficLight.state values, from the message constants (docs/03 §2.1).
enum class TrafficLightColor : std::uint8_t {
  kUnknown = nuway_msgs::msg::TrafficLight::STATE_UNKNOWN,
  kRed = nuway_msgs::msg::TrafficLight::STATE_RED,
  kYellow = nuway_msgs::msg::TrafficLight::STATE_YELLOW,
  kGreen = nuway_msgs::msg::TrafficLight::STATE_GREEN,
  kOff = nuway_msgs::msg::TrafficLight::STATE_OFF,
};

// One observed traffic light (nuway_msgs/TrafficLight, docs/02 §4).
struct TrafficLightObs {
  std::uint32_t id = 0;
  TrafficLightColor state = TrafficLightColor::kUnknown;
  Eigen::Vector2d stop_line = Eigen::Vector2d::Zero();  // map frame, m
  std::vector<std::uint32_t> affected_lane_ids;
  double confidence = 1.0;
  double time_in_state_s = -1.0;  // -1 unknown
  double yellow_duration_s = 3.0;
};

// Everything a planning tick sees. Non-owning pointers may be null: a
// missing route line is the no-input case, a missing lane graph only
// disables the lane-based checks.
struct SceneInput {
  EgoObs ego;
  const RouteLine* route = nullptr;
  const nuway_map::LaneGraph* graph = nullptr;
  std::vector<nuway_common::AgentState> agents;  // map frame
  nuway_common::PredictionSet predictions;       // map frame
  std::vector<TrafficLightObs> lights;
  double dt_s = 0.1;  // time since the previous planning tick
};

}  // namespace nuway_planning

#endif  // NUWAY_PLANNING_SCENE_H_
