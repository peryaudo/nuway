// behavior_fsm_node (M1 §3.2): the ROS shell around BehaviorFsm. On every
// planning tick it assembles the SceneInput from the tick's pose, agents
// (transformed into map with that pose, M1 §3.1), predictions and traffic
// lights, plus the latched route line, route and lane graph, and publishes
// /nuway/planning/behavior stamped with the tick.
//
// Current-tick barrier over pose, agents, traffic_lights, samples and
// fallback_samples (docs/02 §2); on TickTimeout(k) the missing inputs are
// degraded until the next ResetEvent. Predictions come from `samples`
// unless it is degraded, then from `fallback_samples` (docs/02 §3.6); a
// degraded pose, agents or lights, or a degraded fallback, yields the
// no-input decision (LONGITUDINAL_STOP, reason no_input). Cross-tick state
// (barrier, buffers, the FSM timers and latches, the route line) is dropped
// on ResetEvent; messages stamped before the reset are ignored (docs/02 §7).
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
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
#include <nuway_msgs/msg/behavior_decision.hpp>
#include <nuway_msgs/msg/ego_state.hpp>
#include <nuway_msgs/msg/lane_graph.hpp>
#include <nuway_msgs/msg/prediction_samples.hpp>
#include <nuway_msgs/msg/reference_line.hpp>
#include <nuway_msgs/msg/reset_event.hpp>
#include <nuway_msgs/msg/route.hpp>
#include <nuway_msgs/msg/tick_timeout.hpp>
#include <nuway_msgs/msg/traffic_light_array.hpp>

#include "nuway_planning/behavior_fsm.h"
#include "nuway_planning/msg_conv.h"
#include "nuway_planning/names.h"
#include "nuway_planning/route_line.h"
#include "nuway_planning/scene.h"

namespace nuway_planning {
namespace {

using nuway_common::DiagStatus;

constexpr const char* kInputPose = "pose";
constexpr const char* kInputAgents = "agents";
constexpr const char* kInputLights = "traffic_lights";
constexpr const char* kInputSamples = "samples";
constexpr const char* kInputFallback = "fallback_samples";
constexpr std::int64_t kBufferTicks = 4;

class BehaviorFsmNode final : public rclcpp::Node {
 public:
  BehaviorFsmNode()
      : rclcpp::Node(kBehaviorFsmNodeName),
        barrier_({kInputPose, kInputAgents, kInputLights, kInputSamples,
                  kInputFallback}),
        diag_(this) {
    fsm_ = std::make_unique<BehaviorFsm>(DeclareOptions());
    pub_behavior_ = create_publisher<nuway_msgs::msg::BehaviorDecision>(
        nuway_common::kTopicPlanningBehavior, nuway_common::qos::Stream());
    sub_lane_graph_ = create_subscription<nuway_msgs::msg::LaneGraph>(
        nuway_common::kTopicLaneGraph, nuway_common::qos::Latched(),
        [this](nuway_msgs::msg::LaneGraph::ConstSharedPtr msg) {
          graph_ = std::make_unique<nuway_map::LaneGraph>(
              nuway_map::LaneGraph::FromMsg(*msg));
          RCLCPP_INFO(get_logger(), "lane graph: %zu lanes", msg->lanes.size());
        });
    sub_route_ = create_subscription<nuway_msgs::msg::Route>(
        nuway_common::kTopicRoutePlan, nuway_common::qos::Latched(),
        [this](nuway_msgs::msg::Route::ConstSharedPtr msg) {
          OnRoute(std::move(msg));
        });
    sub_reference_line_ = create_subscription<nuway_msgs::msg::ReferenceLine>(
        nuway_common::kTopicReferenceLine, nuway_common::qos::Latched(),
        [this](nuway_msgs::msg::ReferenceLine::ConstSharedPtr msg) {
          OnReferenceLine(std::move(msg));
        });
    sub_pose_ = create_subscription<nuway_msgs::msg::EgoState>(
        nuway_common::kTopicPose, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::EgoState::ConstSharedPtr msg) {
          OnInput(kInputPose, &poses_, std::move(msg));
        });
    sub_agents_ = create_subscription<nuway_msgs::msg::AgentArray>(
        nuway_common::kTopicPerceptionAgents, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::AgentArray::ConstSharedPtr msg) {
          OnInput(kInputAgents, &agents_, std::move(msg));
        });
    sub_lights_ = create_subscription<nuway_msgs::msg::TrafficLightArray>(
        nuway_common::kTopicPerceptionTrafficLights,
        nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::TrafficLightArray::ConstSharedPtr msg) {
          OnInput(kInputLights, &lights_, std::move(msg));
        });
    sub_samples_ = create_subscription<nuway_msgs::msg::PredictionSamples>(
        nuway_common::kTopicPredictionSamples, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::PredictionSamples::ConstSharedPtr msg) {
          OnInput(kInputSamples, &samples_, std::move(msg));
        });
    sub_fallback_ = create_subscription<nuway_msgs::msg::PredictionSamples>(
        nuway_common::kTopicPredictionFallbackSamples,
        nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::PredictionSamples::ConstSharedPtr msg) {
          OnInput(kInputFallback, &fallback_, std::move(msg));
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
  // Declares every BehaviorFsmOptions field as a parameter (docs/03 §4).
  BehaviorFsmOptions DeclareOptions() {
    BehaviorFsmOptions o;
    const auto d = [this](const char* name, double value, const char* doc) {
      return nuway_common::DeclareParam<double>(this, name, value, doc);
    };
    o.overtake_enabled = nuway_common::DeclareParam<bool>(
        this, "overtake_enabled", o.overtake_enabled, "overtake a slow lead");
    o.slow_lead_margin_mps =
        d("slow_lead_margin_mps", o.slow_lead_margin_mps, "slow lead margin");
    o.slow_lead_s = d("slow_lead_s", o.slow_lead_s, "slow lead duration");
    o.gap_back_m = d("gap_back_m", o.gap_back_m, "gap behind");
    o.gap_ahead_m = d("gap_ahead_m", o.gap_ahead_m, "gap ahead");
    o.gap_horizon_s = d("gap_horizon_s", o.gap_horizon_s, "gap horizon");
    o.change_commit_s = d("change_commit_s", o.change_commit_s, "commit time");
    o.abort_gap_m = d("abort_gap_m", o.abort_gap_m, "abort gap");
    o.follow_range_m = d("follow_range_m", o.follow_range_m, "lead range");
    o.lane_slack_m = d("lane_slack_m", o.lane_slack_m, "lane band slack");
    o.idm_s0_m = d("idm_s0_m", o.idm_s0_m, "IDM jam distance");
    o.idm_t_s = d("idm_t_s", o.idm_t_s, "IDM headway");
    o.idm_a_mps2 = d("idm_a_mps2", o.idm_a_mps2, "IDM accel");
    o.idm_b_mps2 = d("idm_b_mps2", o.idm_b_mps2, "IDM decel");
    o.idm_horizon_s = d("idm_horizon_s", o.idm_horizon_s, "IDM horizon");
    o.a_comf_mps2 = d("a_comf_mps2", o.a_comf_mps2, "comfortable braking");
    o.t_react_s = d("t_react_s", o.t_react_s, "reaction time");
    o.stop_lookahead_m =
        d("stop_lookahead_m", o.stop_lookahead_m, "stop distance slack");
    o.stop_margin_m = d("stop_margin_m", o.stop_margin_m, "stop_s margin");
    o.passed_line_m = d("passed_line_m", o.passed_line_m, "passed line");
    o.unknown_confidence =
        d("unknown_confidence", o.unknown_confidence, "unknown TL threshold");
    o.stop_sign_speed_mps =
        d("stop_sign_speed_mps", o.stop_sign_speed_mps, "stop sign speed");
    o.stop_sign_dist_m =
        d("stop_sign_dist_m", o.stop_sign_dist_m, "stop sign distance");
    o.stop_sign_hold_s = d("stop_sign_hold_s", o.stop_sign_hold_s, "stop hold");
    o.yield_horizon_s =
        d("yield_horizon_s", o.yield_horizon_s, "yield horizon");
    o.yield_lookahead_m =
        d("yield_lookahead_m", o.yield_lookahead_m, "yield lookahead");
    o.yield_margin_m = d("yield_margin_m", o.yield_margin_m, "yield margin");
    o.yield_time_margin_s =
        d("yield_time_margin_s", o.yield_time_margin_s, "yield time margin");
    o.crossing_angle_rad =
        d("crossing_angle_rad", o.crossing_angle_rad, "crossing angle");
    o.yield_stop_back_m =
        d("yield_stop_back_m", o.yield_stop_back_m, "yield stop back");
    o.yield_hold_s = d("yield_hold_s", o.yield_hold_s, "yield hysteresis");
    o.speed.a_lat_max_mps2 =
        d("a_lat_max_mps2", o.speed.a_lat_max_mps2, "lateral accel bound");
    o.speed.plan_decel_mps2 =
        d("plan_decel_mps2", o.speed.plan_decel_mps2, "profile braking");
    o.speed.curvature_horizon_m = d(
        "curvature_horizon_m", o.speed.curvature_horizon_m, "profile horizon");
    o.projection_max_dist_m =
        d("projection_max_dist_m", o.projection_max_dist_m, "off-line limit");
    o.projection_back_m =
        d("projection_back_m", o.projection_back_m, "projection window back");
    o.projection_ahead_m = d("projection_ahead_m", o.projection_ahead_m,
                             "projection window ahead");
    o.lane_search_dist_m =
        d("lane_search_dist_m", o.lane_search_dist_m, "NearestLane radius");
    o.default_lane_half_width_m = d("default_lane_half_width_m",
                                    o.default_lane_half_width_m, "half width");
    o.ego_front_m = d("ego_front_m", o.ego_front_m, "rear axle to bumper");
    return o;
  }

  template <typename MsgT>
  static void Prune(std::map<std::int64_t, MsgT>* buffer, std::int64_t k) {
    while (!buffer->empty() && buffer->begin()->first < k - kBufferTicks) {
      buffer->erase(buffer->begin());
    }
  }

  // The tick's message in a buffer, or nullptr.
  template <typename MsgT>
  static const MsgT* Find(const std::map<std::int64_t, MsgT>& buffer,
                          std::int64_t k) {
    const auto it = buffer.find(k);
    return it == buffer.end() ? nullptr : &it->second;
  }

  bool Accepts(std::int64_t k) const {
    return k >= episode_start_k_ && nuway_common::IsPlanningTick(k);
  }

  // Before the current episode, compared as tick indices (docs/02 §7).
  bool BeforeEpisode(const builtin_interfaces::msg::Time& stamp) const {
    return nuway_common::TickIndex(stamp) < episode_start_k_ &&
           rclcpp::Time(stamp).nanoseconds() != 0;
  }

  // Buffers a per-tick input and runs the tick once the barrier completes.
  template <typename MsgT>
  void OnInput(const char* name, std::map<std::int64_t, MsgT>* buffer,
               MsgT msg) {
    const std::int64_t k = nuway_common::TickIndex(msg->header.stamp);
    if (!Accepts(k)) {
      return;
    }
    (*buffer)[k] = std::move(msg);
    Prune(buffer, k);
    barrier_.Arrive(name, k);
    if (barrier_.IsComplete(k)) {
      Run(k);
    }
  }

  void OnRoute(nuway_msgs::msg::Route::ConstSharedPtr msg) {
    if (BeforeEpisode(msg->header.stamp)) {
      return;
    }
    route_msg_ = std::move(msg);
    RebuildRouteLine();
  }

  void OnReferenceLine(nuway_msgs::msg::ReferenceLine::ConstSharedPtr msg) {
    if (BeforeEpisode(msg->header.stamp)) {
      return;
    }
    line_msg_ = std::move(msg);
    RebuildRouteLine();
  }

  void RebuildRouteLine() {
    if (line_msg_ == nullptr) {
      return;
    }
    std::optional<RouteLine> line =
        RouteLineFromMsg(*line_msg_, route_msg_.get());
    if (!line.has_value()) {
      RCLCPP_ERROR(get_logger(), "malformed reference line; ignored");
      return;
    }
    route_ = std::make_unique<RouteLine>(std::move(*line));
    RCLCPP_INFO(get_logger(), "route line: %d points, %.0f m, goal at %.0f m",
                route_->line().size(), route_->length(), route_->goal_s());
  }

  void OnResetEvent(const nuway_msgs::msg::ResetEvent& msg) {
    if (episode_id_.has_value() && msg.episode_id <= *episode_id_) {
      return;  // a replayed (transient_local) event
    }
    episode_id_ = msg.episode_id;
    episode_start_k_ = nuway_common::TickIndex(msg.header.stamp);
    last_published_k_ = episode_start_k_ - 1;
    barrier_.Reset();
    poses_.clear();
    agents_.clear();
    lights_.clear();
    samples_.clear();
    fallback_.clear();
    fsm_->Reset();
    // A route or line stamped for this episode may already have arrived
    // (subscriptions are served in creation order); only older ones go.
    if (line_msg_ != nullptr && BeforeEpisode(line_msg_->header.stamp)) {
      line_msg_.reset();
      route_msg_.reset();
      route_.reset();
    }
    RCLCPP_INFO(get_logger(), "reset: episode %u", msg.episode_id);
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
    BehaviorOutput out = NoInputDecision();
    builtin_interfaces::msg::Time stamp = nuway_common::TickStamp(k);
    DiagStatus status = DiagStatus::kOk;
    std::string message;
    {
      const nuway_common::ScopedTimer timer(&cycle_ms);
      const auto* pose = Find(poses_, k);
      const auto* agents = Find(agents_, k);
      const auto* lights = Find(lights_, k);
      const bool use_fallback = barrier_.IsDegraded(kInputSamples);
      const auto* samples =
          use_fallback ? Find(fallback_, k) : Find(samples_, k);
      if (pose != nullptr) {
        stamp = (*pose)->header.stamp;
      }
      if (pose == nullptr || agents == nullptr || lights == nullptr ||
          samples == nullptr) {
        status = DiagStatus::kWarn;
        message = "missing input for the tick (degraded)";
      } else if (!(*pose)->valid) {
        message = "pose invalid";
      } else if (route_ == nullptr) {
        message = "no route line";
      } else {
        SceneInput in;
        in.ego = EgoObsFromMsg(**pose);
        in.route = route_.get();
        in.graph = graph_.get();
        in.agents = nuway_common::AgentStatesFromMsg(
            nuway_common::AgentsToMap(**agents, **pose));
        in.predictions = nuway_common::PredictionSetFromMsg(**samples);
        in.lights = TrafficLightsFromMsg(**lights);
        in.dt_s = last_step_k_.has_value()
                      ? static_cast<double>(k - *last_step_k_) *
                            nuway_common::kTickDtS
                      : 2.0 * nuway_common::kTickDtS;
        last_step_k_ = k;
        out = fsm_->Step(in);
        if (use_fallback) {
          status = DiagStatus::kWarn;
          message = "samples degraded; using fallback_samples";
        }
      }
    }
    nuway_msgs::msg::BehaviorDecision msg;
    msg.header.stamp = stamp;
    msg.lateral = static_cast<std::uint8_t>(out.lateral);
    msg.target_lane_id = out.target_lane_id;
    msg.longitudinal = static_cast<std::uint8_t>(out.longitudinal);
    msg.lead_agent_id = out.lead_agent_id;
    msg.stop_s = static_cast<float>(out.stop_s);
    msg.target_speed = static_cast<float>(out.target_speed_mps);
    msg.reason = out.reason;
    pub_behavior_->publish(msg);
    diag_.Publish(stamp, cycle_ms, 0.0, status,
                  message.empty() ? out.reason : message);
  }

  std::unique_ptr<BehaviorFsm> fsm_;
  std::unique_ptr<nuway_map::LaneGraph> graph_;
  nuway_msgs::msg::Route::ConstSharedPtr route_msg_;
  nuway_msgs::msg::ReferenceLine::ConstSharedPtr line_msg_;
  std::unique_ptr<RouteLine> route_;

  nuway_common::TickBarrier barrier_;
  std::optional<std::uint32_t> episode_id_;
  std::int64_t episode_start_k_ = 0;
  std::int64_t last_published_k_ = -1;
  std::optional<std::int64_t> last_step_k_;
  std::map<std::int64_t, nuway_msgs::msg::EgoState::ConstSharedPtr> poses_;
  std::map<std::int64_t, nuway_msgs::msg::AgentArray::ConstSharedPtr> agents_;
  std::map<std::int64_t, nuway_msgs::msg::TrafficLightArray::ConstSharedPtr>
      lights_;
  std::map<std::int64_t, nuway_msgs::msg::PredictionSamples::ConstSharedPtr>
      samples_;
  std::map<std::int64_t, nuway_msgs::msg::PredictionSamples::ConstSharedPtr>
      fallback_;

  nuway_common::DiagPublisher diag_;
  rclcpp::Publisher<nuway_msgs::msg::BehaviorDecision>::SharedPtr pub_behavior_;
  rclcpp::Subscription<nuway_msgs::msg::LaneGraph>::SharedPtr sub_lane_graph_;
  rclcpp::Subscription<nuway_msgs::msg::Route>::SharedPtr sub_route_;
  rclcpp::Subscription<nuway_msgs::msg::ReferenceLine>::SharedPtr
      sub_reference_line_;
  rclcpp::Subscription<nuway_msgs::msg::EgoState>::SharedPtr sub_pose_;
  rclcpp::Subscription<nuway_msgs::msg::AgentArray>::SharedPtr sub_agents_;
  rclcpp::Subscription<nuway_msgs::msg::TrafficLightArray>::SharedPtr
      sub_lights_;
  rclcpp::Subscription<nuway_msgs::msg::PredictionSamples>::SharedPtr
      sub_samples_;
  rclcpp::Subscription<nuway_msgs::msg::PredictionSamples>::SharedPtr
      sub_fallback_;
  rclcpp::Subscription<nuway_msgs::msg::ResetEvent>::SharedPtr sub_reset_;
  rclcpp::Subscription<nuway_msgs::msg::TickTimeout>::SharedPtr
      sub_tick_timeout_;
};

}  // namespace
}  // namespace nuway_planning

int main(int argc, char** argv) {
  return nuway_common::RunNode<nuway_planning::BehaviorFsmNode>(argc, argv);
}
