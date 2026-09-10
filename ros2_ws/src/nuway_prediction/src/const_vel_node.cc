// const_vel_node (M1 §3.1): the ROS shell around ConstVelPredictor. On every
// planning tick it turns /nuway/perception/agents (base_link, transformed
// into map with the EgoState of the same tick through AgentsToMap, the
// frame rule of M1 §3.1) into /nuway/prediction/fallback_samples, and into
// /nuway/prediction/samples as well when it is the primary producer
// (`publish_primary`, set by the launch file from prediction.source). It
// runs in every profile: from M7 on a dead learned predictor degrades
// `samples` and the consumers switch to this node's `fallback_samples`
// (docs/02 §2 Degradation) without any timer of their own.
//
// Current-tick barrier over the pose and the agents (docs/02 §2): the node
// acts on even ticks only, runs once both inputs stamped k are in, and on
// TickTimeout(k) degrades the missing one until the next ResetEvent. The
// no-input convention: a missing or invalid pose, or degraded agents,
// yield a PredictionSamples with zero agents stamped with the tick. The
// lane graph is latched and read latest-value. Cross-tick state (barrier,
// buffers) is dropped on ResetEvent; messages stamped before the reset are
// ignored (docs/02 §7). The predictor itself is stateless.
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <nuway_common/agents.h>
#include <nuway_common/diag.h>
#include <nuway_common/frames.h>
#include <nuway_common/node_main.h>
#include <nuway_common/params.h>
#include <nuway_common/qos.h>
#include <nuway_common/ros_conv.h>
#include <nuway_common/tick.h>
#include <nuway_map/lane_graph.h>
#include <nuway_msgs/msg/agent_array.hpp>
#include <nuway_msgs/msg/ego_state.hpp>
#include <nuway_msgs/msg/lane_graph.hpp>
#include <nuway_msgs/msg/prediction_samples.hpp>
#include <nuway_msgs/msg/reset_event.hpp>
#include <nuway_msgs/msg/tick_timeout.hpp>

#include "nuway_prediction/const_vel.h"
#include "nuway_prediction/names.h"

namespace nuway_prediction {
namespace {

using nuway_common::DiagStatus;

constexpr const char* kInputPose = "pose";
constexpr const char* kInputAgents = "agents";
// Buffered inputs older than this many ticks behind the newest are dropped.
constexpr std::int64_t kBufferTicks = 4;

class ConstVelNode final : public rclcpp::Node {
 public:
  ConstVelNode()
      : rclcpp::Node(kConstVelNodeName),
        barrier_({kInputPose, kInputAgents}),
        diag_(this) {
    ConstVelOptions options;
    publish_primary_ = nuway_common::DeclareParam<bool>(
        this, "publish_primary", true,
        "also publish /nuway/prediction/samples (the primary producer)");
    options.lane_follow = nuway_common::DeclareParam<bool>(
        this, "lane_follow", options.lane_follow,
        "vehicles follow their lane instead of a straight line");
    options.num_timesteps = nuway_common::DeclareParam<int>(
        this, "num_timesteps", options.num_timesteps, "T prediction steps");
    options.dt_s = nuway_common::DeclareParam<double>(this, "dt_s",
                                                      options.dt_s, "step, s");
    options.pedestrian_speed_max_mps = nuway_common::DeclareParam<double>(
        this, "pedestrian_speed_max_mps", options.pedestrian_speed_max_mps,
        "walker speed clamp");
    options.lane_search_dist_m = nuway_common::DeclareParam<double>(
        this, "lane_search_dist_m", options.lane_search_dist_m,
        "NearestLane radius for lane_follow");
    options.lane_follow_min_speed_mps = nuway_common::DeclareParam<double>(
        this, "lane_follow_min_speed_mps", options.lane_follow_min_speed_mps,
        "slower along the lane: straight line");
    predictor_ = std::make_unique<ConstVelPredictor>(options);
    RCLCPP_INFO(get_logger(),
                "const_vel: T=%d dt=%.2f lane_follow=%d publish_primary=%d",
                options.num_timesteps, options.dt_s,
                static_cast<int>(options.lane_follow),
                static_cast<int>(publish_primary_));

    pub_fallback_ = create_publisher<nuway_msgs::msg::PredictionSamples>(
        nuway_common::kTopicPredictionFallbackSamples,
        nuway_common::qos::Stream());
    if (publish_primary_) {
      pub_samples_ = create_publisher<nuway_msgs::msg::PredictionSamples>(
          nuway_common::kTopicPredictionSamples, nuway_common::qos::Stream());
    }
    sub_lane_graph_ = create_subscription<nuway_msgs::msg::LaneGraph>(
        nuway_common::kTopicLaneGraph, nuway_common::qos::Latched(),
        [this](nuway_msgs::msg::LaneGraph::ConstSharedPtr msg) {
          OnLaneGraph(*msg);
        });
    sub_pose_ = create_subscription<nuway_msgs::msg::EgoState>(
        nuway_common::kTopicPose, nuway_common::qos::Stream(),
        [this](const nuway_msgs::msg::EgoState& msg) { OnPose(msg); });
    sub_agents_ = create_subscription<nuway_msgs::msg::AgentArray>(
        nuway_common::kTopicPerceptionAgents, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::AgentArray::ConstSharedPtr msg) {
          OnAgents(std::move(msg));
        });
    sub_reset_ = create_subscription<nuway_msgs::msg::ResetEvent>(
        nuway_common::kTopicResetEvent, nuway_common::qos::Event(),
        [this](const nuway_msgs::msg::ResetEvent& msg) { OnResetEvent(msg); });
    sub_tick_timeout_ = create_subscription<nuway_msgs::msg::TickTimeout>(
        nuway_common::kTopicTickTimeout, nuway_common::qos::Event(),
        [this](const nuway_msgs::msg::TickTimeout& msg) {
          OnTickTimeout(msg);
        });
  }

 private:
  template <typename MsgT>
  static void Prune(std::map<std::int64_t, MsgT>* buffer, std::int64_t k) {
    while (!buffer->empty() && buffer->begin()->first < k - kBufferTicks) {
      buffer->erase(buffer->begin());
    }
  }

  // A stamp belongs to this node's work when it is in the current episode
  // and on a planning tick (docs/02 §2).
  bool Accepts(std::int64_t k) const {
    return k >= episode_start_k_ && nuway_common::IsPlanningTick(k);
  }

  void OnLaneGraph(const nuway_msgs::msg::LaneGraph& msg) {
    lane_graph_ = std::make_unique<nuway_map::LaneGraph>(
        nuway_map::LaneGraph::FromMsg(msg));
    predictor_->set_lane_graph(lane_graph_.get());
    RCLCPP_INFO(get_logger(), "lane graph: %zu lanes", msg.lanes.size());
  }

  void OnResetEvent(const nuway_msgs::msg::ResetEvent& msg) {
    episode_start_k_ = nuway_common::TickIndex(msg.header.stamp);
    last_published_k_ = episode_start_k_ - 1;
    barrier_.Reset();
    poses_.clear();
    agents_.clear();
    RCLCPP_INFO(get_logger(), "reset: episode %u", msg.episode_id);
  }

  void OnPose(const nuway_msgs::msg::EgoState& msg) {
    const std::int64_t k = nuway_common::TickIndex(msg.header.stamp);
    if (!Accepts(k)) {
      return;
    }
    poses_[k] = msg;
    Prune(&poses_, k);
    barrier_.Arrive(kInputPose, k);
    if (barrier_.IsComplete(k)) {
      Run(k);
    }
  }

  void OnAgents(nuway_msgs::msg::AgentArray::ConstSharedPtr msg) {
    const std::int64_t k = nuway_common::TickIndex(msg->header.stamp);
    if (!Accepts(k)) {
      return;
    }
    agents_[k] = std::move(msg);
    Prune(&agents_, k);
    barrier_.Arrive(kInputAgents, k);
    if (barrier_.IsComplete(k)) {
      Run(k);
    }
  }

  void OnTickTimeout(const nuway_msgs::msg::TickTimeout& msg) {
    const std::int64_t k = nuway_common::TickIndex(msg.header.stamp);
    if (!Accepts(k)) {
      return;
    }
    for (const std::string& name : barrier_.Missing(k)) {
      barrier_.Degrade(name);
      RCLCPP_WARN(get_logger(), "tick %ld timed out; %s degraded until reset",
                  static_cast<long>(k), name.c_str());
    }
    Run(k);
  }

  void Run(std::int64_t k) {
    if (k <= last_published_k_) {
      return;
    }
    last_published_k_ = k;
    double cycle_ms = 0.0;
    nuway_msgs::msg::PredictionSamples out;
    DiagStatus status = DiagStatus::kOk;
    std::string message;
    {
      const nuway_common::ScopedTimer timer(&cycle_ms);
      const auto pose_it = poses_.find(k);
      const auto agents_it = agents_.find(k);
      if (pose_it == poses_.end()) {
        out = NoInput(nuway_common::TickStamp(k));
        status = DiagStatus::kWarn;
        message = "no pose for the tick (degraded)";
      } else if (!pose_it->second.valid) {
        out = NoInput(pose_it->second.header.stamp);
        message = "pose invalid";
      } else if (agents_it == agents_.end()) {
        out = NoInput(pose_it->second.header.stamp);
        status = DiagStatus::kWarn;
        message = "no agents for the tick (degraded)";
      } else {
        const nuway_msgs::msg::AgentArray in_map =
            nuway_common::AgentsToMap(*agents_it->second, pose_it->second);
        out = nuway_common::PredictionSetToMsg(
            predictor_->Predict(nuway_common::AgentStatesFromMsg(in_map)));
        out.header.stamp = pose_it->second.header.stamp;
        if (lane_graph_ == nullptr && predictor_->options().lane_follow) {
          message = "no lane graph yet; straight-line prediction";
        }
      }
    }
    pub_fallback_->publish(out);
    if (pub_samples_ != nullptr) {
      pub_samples_->publish(out);
    }
    diag_.Publish(out.header.stamp, cycle_ms, 0.0, status, message);
  }

  // The no-input output of docs/02 §2: zero agents, one empty sample.
  nuway_msgs::msg::PredictionSamples NoInput(
      const builtin_interfaces::msg::Time& stamp) const {
    nuway_common::PredictionSet empty;
    empty.num_samples = 1;
    empty.num_timesteps = predictor_->options().num_timesteps;
    empty.dt_s = predictor_->options().dt_s;
    empty.sample_weight = {1.0};
    nuway_msgs::msg::PredictionSamples out =
        nuway_common::PredictionSetToMsg(empty);
    out.header.stamp = stamp;
    return out;
  }

  bool publish_primary_ = true;
  std::unique_ptr<ConstVelPredictor> predictor_;
  std::unique_ptr<nuway_map::LaneGraph> lane_graph_;

  nuway_common::TickBarrier barrier_;
  std::int64_t episode_start_k_ = 0;
  std::int64_t last_published_k_ = -1;
  std::map<std::int64_t, nuway_msgs::msg::EgoState> poses_;
  std::map<std::int64_t, nuway_msgs::msg::AgentArray::ConstSharedPtr> agents_;

  nuway_common::DiagPublisher diag_;
  rclcpp::Publisher<nuway_msgs::msg::PredictionSamples>::SharedPtr pub_samples_;
  rclcpp::Publisher<nuway_msgs::msg::PredictionSamples>::SharedPtr
      pub_fallback_;
  rclcpp::Subscription<nuway_msgs::msg::LaneGraph>::SharedPtr sub_lane_graph_;
  rclcpp::Subscription<nuway_msgs::msg::EgoState>::SharedPtr sub_pose_;
  rclcpp::Subscription<nuway_msgs::msg::AgentArray>::SharedPtr sub_agents_;
  rclcpp::Subscription<nuway_msgs::msg::ResetEvent>::SharedPtr sub_reset_;
  rclcpp::Subscription<nuway_msgs::msg::TickTimeout>::SharedPtr
      sub_tick_timeout_;
};

}  // namespace
}  // namespace nuway_prediction

int main(int argc, char** argv) {
  return nuway_common::RunNode<nuway_prediction::ConstVelNode>(argc, argv);
}
