// Message -> scene.h conversions for the planning nodes (M1). Field copies
// only; the geometric work happens in the libraries. Header-only so the
// nodes and (from M6) nuway_py share one definition.
#ifndef NUWAY_PLANNING_MSG_CONV_H_
#define NUWAY_PLANNING_MSG_CONV_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <nuway_common/frenet.h>
#include <nuway_common/geometry.h>
#include <nuway_common/ros_conv.h>
#include <nuway_msgs/msg/behavior_decision.hpp>
#include <nuway_msgs/msg/ego_state.hpp>
#include <nuway_msgs/msg/reference_line.hpp>
#include <nuway_msgs/msg/route.hpp>
#include <nuway_msgs/msg/traffic_light_array.hpp>

#include "nuway_planning/route_line.h"
#include "nuway_planning/scene.h"

namespace nuway_planning {

inline EgoObs EgoObsFromMsg(const nuway_msgs::msg::EgoState& msg) {
  EgoObs out;
  out.pose = nuway_common::SE2FromMsg(msg.pose);
  out.vx_mps = msg.vx;
  out.vy_mps = msg.vy;
  out.ax_mps2 = msg.ax;
  out.ay_mps2 = msg.ay;
  out.yaw_rate_radps = msg.yaw_rate;
  out.steering_angle_rad = msg.steering_angle;
  return out;
}

inline std::vector<TrafficLightObs> TrafficLightsFromMsg(
    const nuway_msgs::msg::TrafficLightArray& msg) {
  std::vector<TrafficLightObs> out;
  out.reserve(msg.lights.size());
  for (const nuway_msgs::msg::TrafficLight& light : msg.lights) {
    TrafficLightObs obs;
    obs.id = light.id;
    obs.state = static_cast<TrafficLightColor>(light.state);
    obs.stop_line = Eigen::Vector2d(light.stop_line.x, light.stop_line.y);
    obs.affected_lane_ids = light.affected_lane_ids;
    obs.confidence = static_cast<double>(light.confidence);
    obs.time_in_state_s = static_cast<double>(light.time_in_state);
    obs.yellow_duration_s = static_cast<double>(light.yellow_duration);
    out.push_back(obs);
  }
  return out;
}

// Builds the RouteLine of an episode from the latched line and, when the
// route message is at hand, its goal projected onto the line. Returns
// nullopt when the message is malformed (fewer than two points or
// mismatched attribute sizes), so a node never indexes past the end.
inline std::optional<RouteLine> RouteLineFromMsg(
    const nuway_msgs::msg::ReferenceLine& msg,
    const nuway_msgs::msg::Route* route) {
  const std::size_t n = msg.points.size();
  if (n < 2 || msg.s.size() != n || msg.heading.size() != n ||
      msg.curvature.size() != n) {
    return std::nullopt;
  }
  nuway_common::Vector2dList points;
  points.reserve(n);
  for (const geometry_msgs::msg::Point& p : msg.points) {
    points.emplace_back(p.x, p.y);
  }
  nuway_common::ReferenceLine line = nuway_common::ReferenceLine::FromSamples(
      points, std::vector<double>(msg.s.begin(), msg.s.end()),
      std::vector<double>(msg.heading.begin(), msg.heading.end()),
      std::vector<double>(msg.curvature.begin(), msg.curvature.end()));
  std::optional<double> goal_s;
  if (route != nullptr) {
    const std::optional<nuway_common::FrenetPoint> goal =
        line.ToFrenet(route->goal.position.x, route->goal.position.y, 20.0);
    if (goal.has_value()) {
      goal_s = goal->s;
    }
  }
  return RouteLine(
      std::move(line), msg.lane_id,
      std::vector<double>(msg.speed_limit.begin(), msg.speed_limit.end()),
      std::vector<double>(msg.left_bound.begin(), msg.left_bound.end()),
      std::vector<double>(msg.right_bound.begin(), msg.right_bound.end()),
      goal_s);
}

}  // namespace nuway_planning

#endif  // NUWAY_PLANNING_MSG_CONV_H_
