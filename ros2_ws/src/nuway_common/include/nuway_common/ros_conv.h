// geometry_msgs <-> Eigen / SE2 / SE3 conversions for nodes (part of
// nuway_common_ros; mirrors nuway_rclpy/ros_conv.py), plus the message <->
// plain-struct copies of agents.h and the one base_link -> map agent
// transform of M1 §3.1 (AgentsToMap). Field copies and one rigid transform;
// nothing here needs a parity test. Introduced in M0; agents in M1.
#ifndef NUWAY_COMMON_ROS_CONV_H_
#define NUWAY_COMMON_ROS_CONV_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <nuway_msgs/msg/agent.hpp>
#include <nuway_msgs/msg/agent_array.hpp>
#include <nuway_msgs/msg/ego_state.hpp>
#include <nuway_msgs/msg/prediction_samples.hpp>
#include <nuway_msgs/msg/trajectory.hpp>
#include <nuway_msgs/msg/trajectory_point.hpp>

#include "nuway_common/agents.h"
#include "nuway_common/frames.h"
#include "nuway_common/geometry.h"
#include "nuway_common/trajectory.h"

namespace nuway_common {

inline geometry_msgs::msg::Point PointMsg(const Eigen::Vector3d& p) {
  geometry_msgs::msg::Point out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}

inline Eigen::Vector3d PointFromMsg(const geometry_msgs::msg::Point& p) {
  return Eigen::Vector3d{p.x, p.y, p.z};
}

inline geometry_msgs::msg::Quaternion QuaternionMsg(
    const Eigen::Quaterniond& q) {
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

inline Eigen::Quaterniond QuaternionFromMsg(
    const geometry_msgs::msg::Quaternion& q) {
  return Eigen::Quaterniond{q.w, q.x, q.y, q.z};
}

inline geometry_msgs::msg::Pose PoseMsg(const SE3& pose) {
  geometry_msgs::msg::Pose out;
  out.position = PointMsg(pose.translation);
  out.orientation = QuaternionMsg(pose.rotation);
  return out;
}

inline SE3 SE3FromMsg(const geometry_msgs::msg::Pose& pose) {
  return SE3{PointFromMsg(pose.position), QuaternionFromMsg(pose.orientation)};
}

inline SE2 SE2FromMsg(const geometry_msgs::msg::Pose& pose) {
  return SE2{pose.position.x, pose.position.y,
             QuaternionToYaw(QuaternionFromMsg(pose.orientation))};
}

// Pose message of a planar pose (z = 0, yaw-only rotation).
inline geometry_msgs::msg::Pose PoseMsg(const SE2& pose) {
  SE3 pose3;
  pose3.translation = Eigen::Vector3d{pose.x, pose.y, 0.0};
  pose3.rotation = YawToQuaternion(pose.yaw);
  return PoseMsg(pose3);
}

// The agent frame rule of M1 §3.1: /nuway/perception/agents arrives in
// base_link and every consumer transforms it into map with the EgoState of
// the same tick, which it already holds under the barrier, never with a TF
// lookup (whose buffer contents depend on delivery timing). Poses are
// composed as SE3 (the box centre keeps its height), velocities and the
// history are rotated / transformed in the plane. An array already in map
// (the GT topic the M6 expert reads) is returned unchanged, so the runtime
// and the expert cannot disagree about a frame.
inline nuway_msgs::msg::AgentArray AgentsToMap(
    const nuway_msgs::msg::AgentArray& agents,
    const nuway_msgs::msg::EgoState& ego) {
  if (agents.header.frame_id == kFrameMap) {
    return agents;
  }
  const SE3 ego_pose = SE3FromMsg(ego.pose);
  const SE2 ego_plane = ToSE2(ego_pose);
  nuway_msgs::msg::AgentArray out = agents;
  out.header.frame_id = kFrameMap;
  for (nuway_msgs::msg::Agent& agent : out.agents) {
    agent.pose = PoseMsg(Compose(ego_pose, SE3FromMsg(agent.pose)));
    const Eigen::Vector2d velocity =
        Rotate(ego_plane, Eigen::Vector2d(static_cast<double>(agent.vx),
                                          static_cast<double>(agent.vy)));
    agent.vx = static_cast<float>(velocity.x());
    agent.vy = static_cast<float>(velocity.y());
    const int n =
        std::min<int>(agent.history_len, nuway_msgs::msg::Agent::HISTORY_LEN);
    for (int i = 0; i < n; ++i) {
      const auto base = static_cast<std::size_t>(i) * 3;
      const Eigen::Vector2d p =
          Apply(ego_plane,
                Eigen::Vector2d(static_cast<double>(agent.history[base]),
                                static_cast<double>(agent.history[base + 1])));
      agent.history[base] = static_cast<float>(p.x());
      agent.history[base + 1] = static_cast<float>(p.y());
      agent.history[base + 2] = static_cast<float>(WrapAngle(
          static_cast<double>(agent.history[base + 2]) + ego_plane.yaw));
    }
  }
  return out;
}

// Plain-struct copies of an AgentArray (whatever its frame; the caller has
// applied AgentsToMap first when a map-frame state is needed).
inline std::vector<AgentState> AgentStatesFromMsg(
    const nuway_msgs::msg::AgentArray& agents) {
  std::vector<AgentState> out;
  out.reserve(agents.agents.size());
  for (const nuway_msgs::msg::Agent& agent : agents.agents) {
    AgentState state;
    state.id = agent.id;
    state.class_id = static_cast<AgentClass>(agent.class_id);
    state.pose = SE2FromMsg(agent.pose);
    state.length_m = static_cast<double>(agent.length);
    state.width_m = static_cast<double>(agent.width);
    state.vx_mps = static_cast<double>(agent.vx);
    state.vy_mps = static_cast<double>(agent.vy);
    state.yaw_rate_radps = static_cast<double>(agent.yaw_rate);
    state.visible = agent.visible;
    out.push_back(state);
  }
  return out;
}

// PredictionSamples -> PredictionSet. A message whose array sizes do not
// match S * A * T is returned with num_samples = 0 (an empty set) rather
// than trusted, so a malformed message can never index out of bounds.
inline PredictionSet PredictionSetFromMsg(
    const nuway_msgs::msg::PredictionSamples& msg) {
  PredictionSet out;
  out.agent_ids = msg.agent_ids;
  out.num_samples = msg.num_samples;
  out.num_timesteps = msg.num_timesteps;
  out.dt_s = static_cast<double>(msg.dt);
  const std::size_t n = static_cast<std::size_t>(out.num_samples) *
                        msg.agent_ids.size() *
                        static_cast<std::size_t>(out.num_timesteps);
  if (msg.xy.size() != 2 * n || msg.yaw.size() != n ||
      msg.sample_weight.size() != static_cast<std::size_t>(out.num_samples)) {
    out.num_samples = 0;
    out.agent_ids.clear();
    return out;
  }
  out.xy.assign(msg.xy.begin(), msg.xy.end());
  out.yaw.assign(msg.yaw.begin(), msg.yaw.end());
  out.sample_weight.assign(msg.sample_weight.begin(), msg.sample_weight.end());
  return out;
}

// PredictionSet -> PredictionSamples (frame_id map; the caller stamps it).
inline nuway_msgs::msg::PredictionSamples PredictionSetToMsg(
    const PredictionSet& set) {
  nuway_msgs::msg::PredictionSamples msg;
  msg.header.frame_id = kFrameMap;
  msg.agent_ids = set.agent_ids;
  msg.num_samples = static_cast<std::uint8_t>(set.num_samples);
  msg.num_timesteps = static_cast<std::uint8_t>(set.num_timesteps);
  msg.dt = static_cast<float>(set.dt_s);
  msg.xy.assign(set.xy.begin(), set.xy.end());
  msg.yaw.assign(set.yaw.begin(), set.yaw.end());
  msg.sample_weight.assign(set.sample_weight.begin(), set.sample_weight.end());
  return msg;
}

// nuway_msgs/Trajectory points -> Trajectory (t relative to the stamp).
inline Trajectory TrajectoryFromMsg(const nuway_msgs::msg::Trajectory& msg) {
  Trajectory out;
  out.reserve(msg.points.size());
  for (const nuway_msgs::msg::TrajectoryPoint& p : msg.points) {
    TrajectoryPoint q;
    q.t = static_cast<double>(p.t);
    q.x = static_cast<double>(p.x);
    q.y = static_cast<double>(p.y);
    q.yaw = static_cast<double>(p.yaw);
    q.v = static_cast<double>(p.v);
    q.a = static_cast<double>(p.a);
    q.kappa = static_cast<double>(p.kappa);
    out.push_back(q);
  }
  return out;
}

// Trajectory -> nuway_msgs/Trajectory in the map frame with the given
// stamp, source and candidate id (docs/02 §4; sample_index -1).
inline nuway_msgs::msg::Trajectory TrajectoryToMsg(
    const Trajectory& traj, const builtin_interfaces::msg::Time& stamp,
    const std::string& source, std::uint32_t candidate_id) {
  nuway_msgs::msg::Trajectory msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = kFrameMap;
  msg.source = source;
  msg.candidate_id = candidate_id;
  msg.sample_index = -1;
  msg.points.reserve(traj.size());
  for (const TrajectoryPoint& p : traj) {
    nuway_msgs::msg::TrajectoryPoint q;
    q.t = static_cast<float>(p.t);
    q.x = static_cast<float>(p.x);
    q.y = static_cast<float>(p.y);
    q.yaw = static_cast<float>(p.yaw);
    q.v = static_cast<float>(p.v);
    q.a = static_cast<float>(p.a);
    q.kappa = static_cast<float>(p.kappa);
    msg.points.push_back(q);
  }
  return msg;
}

}  // namespace nuway_common

#endif  // NUWAY_COMMON_ROS_CONV_H_
