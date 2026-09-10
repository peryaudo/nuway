#include "nuway_prediction/const_vel.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <Eigen/Core>

#include <nuway_common/frenet.h>

namespace nuway_prediction {

using nuway_common::AgentClass;
using nuway_common::AgentState;
using nuway_common::PredictionSet;
using nuway_common::SE2;

ConstVelPredictor::ConstVelPredictor(ConstVelOptions options)
    : options_(options) {}

PredictionSet ConstVelPredictor::Predict(
    const std::vector<AgentState>& agents) const {
  PredictionSet out;
  out.num_samples = 1;
  out.num_timesteps = options_.num_timesteps;
  out.dt_s = options_.dt_s;
  out.sample_weight = {1.0};
  out.agent_ids.reserve(agents.size());
  const auto steps = static_cast<std::size_t>(options_.num_timesteps);
  out.xy.reserve(2 * steps * agents.size());
  out.yaw.reserve(steps * agents.size());
  for (const AgentState& agent : agents) {
    const AgentFuture future = PredictAgent(agent);
    out.agent_ids.push_back(agent.id);
    for (const SE2& pose : future.poses) {
      out.xy.push_back(pose.x);
      out.xy.push_back(pose.y);
      out.yaw.push_back(pose.yaw);
    }
  }
  return out;
}

AgentFuture ConstVelPredictor::PredictAgent(const AgentState& agent) const {
  switch (agent.class_id) {
    case AgentClass::kPedestrian:
      return Straight(agent, options_.pedestrian_speed_max_mps);
    case AgentClass::kStaticObstacle:
      return Straight(agent, 0.0);
    default:
      break;
  }
  if (options_.lane_follow && graph_ != nullptr &&
      nuway_common::IsVehicle(agent.class_id)) {
    const std::optional<AgentFuture> along = AlongLane(agent);
    if (along.has_value()) {
      return *along;
    }
  }
  return Straight(agent, -1.0);
}

AgentFuture ConstVelPredictor::Straight(const AgentState& agent,
                                        double speed_cap_mps) const {
  double vx = agent.vx_mps;
  double vy = agent.vy_mps;
  const double speed = agent.speed_mps();
  if (speed_cap_mps >= 0.0 && speed > speed_cap_mps) {
    const double scale = speed > 0.0 ? speed_cap_mps / speed : 0.0;
    vx *= scale;
    vy *= scale;
  }
  AgentFuture out;
  out.id = agent.id;
  out.poses.reserve(static_cast<std::size_t>(options_.num_timesteps));
  for (int t = 0; t < options_.num_timesteps; ++t) {
    const double dt = static_cast<double>(t + 1) * options_.dt_s;
    out.poses.push_back(SE2{agent.pose.x + (vx * dt), agent.pose.y + (vy * dt),
                            agent.pose.yaw});
  }
  return out;
}

std::uint32_t ConstVelPredictor::SmoothestSuccessor(std::uint32_t lane_id,
                                                    double heading_rad) const {
  std::uint32_t best = 0;
  double best_turn = 1e9;
  for (const std::uint32_t next_id : graph_->Successors(lane_id)) {
    const nuway_common::ReferenceLine* next = graph_->reference_line(next_id);
    if (next == nullptr || next->empty()) {
      continue;
    }
    const double turn =
        std::abs(nuway_common::WrapAngle(next->HeadingAt(0.0) - heading_rad));
    // Strictly smaller keeps the lowest id on a tie: the successor list is
    // built in lane-id order, so the choice is the same in every process.
    if (turn < best_turn) {
      best_turn = turn;
      best = next_id;
    }
  }
  return best;
}

std::optional<AgentFuture> ConstVelPredictor::AlongLane(
    const AgentState& agent) const {
  const std::optional<nuway_map::LaneQuery> query = graph_->NearestLane(
      agent.pose.x, agent.pose.y, agent.pose.yaw, options_.lane_search_dist_m);
  if (!query.has_value()) {
    return std::nullopt;
  }
  const nuway_common::ReferenceLine* line =
      graph_->reference_line(query->lane_id);
  if (line == nullptr || line->size() < 2) {
    return std::nullopt;
  }
  // Speed along the lane: the velocity projected onto the lane tangent at
  // the foot point. Against the lane (reversing) or crawling: the straight
  // line is at least as good and never walks the agent through a junction
  // it is not driving into.
  const double heading = line->HeadingAt(query->s);
  const double v_along =
      (agent.vx_mps * std::cos(heading)) + (agent.vy_mps * std::sin(heading));
  if (v_along < options_.lane_follow_min_speed_mps) {
    return std::nullopt;
  }
  AgentFuture out;
  out.id = agent.id;
  out.poses.reserve(static_cast<std::size_t>(options_.num_timesteps));
  const double d = query->d;
  double s = query->s;
  std::uint32_t lane_id = query->lane_id;
  // Past the last lane with no successor the walk continues straight from
  // the line end along its final heading, `overflow` metres beyond it.
  double overflow = 0.0;
  for (int t = 0; t < options_.num_timesteps; ++t) {
    if (overflow > 0.0) {
      overflow += v_along * options_.dt_s;
    } else {
      s += v_along * options_.dt_s;
      while (s > line->length()) {
        const std::uint32_t next_id =
            SmoothestSuccessor(lane_id, line->HeadingAt(line->length()));
        const nuway_common::ReferenceLine* next =
            next_id == 0 ? nullptr : graph_->reference_line(next_id);
        if (next == nullptr || next->size() < 2) {
          overflow = s - line->length();
          s = line->length();
          break;
        }
        s -= line->length();
        lane_id = next_id;
        line = next;
      }
    }
    const nuway_common::CartesianPoint on_line =
        line->ToCartesian(nuway_common::FrenetPoint{s, d});
    SE2 pose{on_line.x, on_line.y, on_line.heading};
    if (overflow > 0.0) {
      pose.x += overflow * std::cos(on_line.heading);
      pose.y += overflow * std::sin(on_line.heading);
    }
    out.poses.push_back(pose);
  }
  return out;
}

}  // namespace nuway_prediction
