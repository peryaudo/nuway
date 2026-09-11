// planner_node (M1 §3.3-§3.6): the ROS shell around Planner. On every
// planning tick it assembles the SceneInput from the tick's pose, agents
// (transformed into map with that pose, §3.1) and predictions plus the
// latched route line, hands the tick's BehaviorDecision to the planner,
// and publishes /nuway/planning/candidates (scored, for the viz) and
// /nuway/planning/trajectory (the selection) stamped with the tick.
//
// Current-tick barrier over pose, agents, behavior, samples and
// fallback_samples (docs/02 §2); on TickTimeout(k) the missing inputs are
// degraded until the next ResetEvent. The three cases of §3.6: a valid
// pose, a line and a usable decision plan the lattice; a valid pose and a
// line without a usable decision (no-input decision, or behavior /
// agents / predictions degraded) plan the injected stop pair alone; no
// line or an invalid pose publish the no-input pair (empty candidates and
// a source "none" stop at the current pose). Cross-tick state goes on
// ResetEvent; messages stamped before the reset are ignored (docs/02 §7).
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
#include <nuway_common/trajectory.h>
#include <nuway_control/vehicle_model.h>
#include <nuway_msgs/msg/agent_array.hpp>
#include <nuway_msgs/msg/behavior_decision.hpp>
#include <nuway_msgs/msg/ego_state.hpp>
#include <nuway_msgs/msg/prediction_samples.hpp>
#include <nuway_msgs/msg/reference_line.hpp>
#include <nuway_msgs/msg/reset_event.hpp>
#include <nuway_msgs/msg/route.hpp>
#include <nuway_msgs/msg/tick_timeout.hpp>
#include <nuway_msgs/msg/trajectory.hpp>
#include <nuway_msgs/msg/trajectory_candidates.hpp>

#include "nuway_planning/msg_conv.h"
#include "nuway_planning/names.h"
#include "nuway_planning/planner.h"
#include "nuway_planning/route_line.h"
#include "nuway_planning/scene.h"

namespace nuway_planning {
namespace {

using nuway_common::DiagStatus;

constexpr const char* kInputPose = "pose";
constexpr const char* kInputAgents = "agents";
constexpr const char* kInputBehavior = "behavior";
constexpr const char* kInputSamples = "samples";
constexpr const char* kInputFallback = "fallback_samples";
constexpr std::int64_t kBufferTicks = 4;

class PlannerNode final : public rclcpp::Node {
 public:
  PlannerNode()
      : rclcpp::Node(kPlannerNodeName),
        barrier_({kInputPose, kInputAgents, kInputBehavior, kInputSamples,
                  kInputFallback}),
        diag_(this) {
    std::optional<PlannerOptions> options = DeclareOptions();
    if (!options.has_value()) {
      // A planner without a vehicle model has no curvature or footprint
      // to plan with; failing loudly at start-up is the better outcome.
      rclcpp::shutdown();
      return;
    }
    planner_ = std::make_unique<Planner>(*options);
    pub_candidates_ = create_publisher<nuway_msgs::msg::TrajectoryCandidates>(
        nuway_common::kTopicPlanningCandidates, nuway_common::qos::Stream());
    pub_trajectory_ = create_publisher<nuway_msgs::msg::Trajectory>(
        nuway_common::kTopicPlanningTrajectory, nuway_common::qos::Stream());
    sub_route_ = create_subscription<nuway_msgs::msg::Route>(
        nuway_common::kTopicRoutePlan, nuway_common::qos::Latched(),
        [this](nuway_msgs::msg::Route::ConstSharedPtr msg) {
          if (!BeforeEpisode(msg->header.stamp)) {
            route_msg_ = std::move(msg);
            RebuildRouteLine();
          }
        });
    sub_reference_line_ = create_subscription<nuway_msgs::msg::ReferenceLine>(
        nuway_common::kTopicReferenceLine, nuway_common::qos::Latched(),
        [this](nuway_msgs::msg::ReferenceLine::ConstSharedPtr msg) {
          if (!BeforeEpisode(msg->header.stamp)) {
            line_msg_ = std::move(msg);
            RebuildRouteLine();
          }
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
    sub_behavior_ = create_subscription<nuway_msgs::msg::BehaviorDecision>(
        nuway_common::kTopicPlanningBehavior, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::BehaviorDecision::ConstSharedPtr msg) {
          OnInput(kInputBehavior, &behaviors_, std::move(msg));
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
  // Declares the parameters (docs/03 §4) and loads the vehicle model;
  // nullopt when the model cannot be loaded.
  std::optional<PlannerOptions> DeclareOptions() {
    const auto d = [this](const char* name, double value, const char* doc) {
      return nuway_common::DeclareParam<double>(this, name, value, doc);
    };
    const auto vehicle_path = nuway_common::DeclareParam<std::string>(
        this, "vehicle", "configs/vehicle/lincoln_mkz_2020.yaml",
        "vehicle model YAML");
    std::string error;
    const std::optional<nuway_control::VehicleModel> model =
        nuway_control::LoadVehicleModel(vehicle_path, &error);
    if (!model.has_value()) {
      RCLCPP_FATAL(get_logger(), "%s", error.c_str());
      return std::nullopt;
    }
    PlannerOptions o;
    o.limits = LatticeLimits::FromVehicleModel(*model);
    o.collision = CollisionOptions::FromVehicleModel(*model);
    o.top_k = nuway_common::DeclareParam<int>(this, "top_k", o.top_k,
                                              "candidates refined by the QPs");
    o.publish_max = nuway_common::DeclareParam<int>(
        this, "publish_max_candidates", o.publish_max,
        "candidates published for the viz");
    SelectorWeights& w = o.weights;
    w.collision = d("rule_selector.w_collision", w.collision, "collision");
    w.ttc = d("rule_selector.w_ttc", w.ttc, "time to collision");
    w.proximity = d("rule_selector.w_proximity", w.proximity, "proximity");
    w.progress = d("rule_selector.w_progress", w.progress, "progress");
    w.speed_dev = d("rule_selector.w_speed_dev", w.speed_dev, "speed dev");
    w.lateral_dev =
        d("rule_selector.w_lateral_dev", w.lateral_dev, "lateral dev");
    w.comfort = d("rule_selector.w_comfort", w.comfort, "comfort");
    w.rule = d("rule_selector.w_rule", w.rule, "rule");
    w.consistency =
        d("rule_selector.w_consistency", w.consistency, "consistency");
    w.qp_relaxed = d("rule_selector.w_qp_relaxed", w.qp_relaxed, "qp relaxed");
    o.collision.margin_lon_m =
        d("collision.margin_lon_m", o.collision.margin_lon_m, "ego lon margin");
    o.collision.margin_lat_m =
        d("collision.margin_lat_m", o.collision.margin_lat_m, "ego lat margin");
    o.collision.agent_margin_m = d("collision.agent_margin_m",
                                   o.collision.agent_margin_m, "agent margin");
    o.lattice.a_gentle_mps2 =
        d("lattice.a_gentle_mps2", o.lattice.a_gentle_mps2, "gentle stop");
    o.lattice.idm_s0_m =
        d("lattice.idm_s0_m", o.lattice.idm_s0_m, "IDM jam gap");
    o.lattice.idm_t_s = d("lattice.idm_t_s", o.lattice.idm_t_s, "IDM headway");
    o.lattice.ds_speed_factor_s = d("lattice.ds_speed_factor_s",
                                    o.lattice.ds_speed_factor_s, "ds factor");
    o.lattice.curvature_cap_factor =
        d("lattice.curvature_cap_factor", o.lattice.curvature_cap_factor,
          "fraction of a_lat_max the keep targets aim at through bends");
    o.lattice.projection_max_dist_m =
        d("lattice.projection_max_dist_m", o.lattice.projection_max_dist_m,
          "off-line limit");
    RefinerOptions& r = o.refiner;
    r.w_d = d("qp.w_d", r.w_d, "path d tracking");
    r.w_dd = d("qp.w_dd", r.w_dd, "path d''");
    r.w_ddd = d("qp.w_ddd", r.w_ddd, "path d'''");
    r.w_end = d("qp.w_end", r.w_end, "path end offset");
    r.w_v = d("qp.w_v", r.w_v, "speed target tracking");
    r.w_a = d("qp.w_a", r.w_a, "speed acceleration");
    r.w_j = d("qp.w_j", r.w_j, "speed jerk");
    r.w_s = d("qp.w_s", r.w_s, "speed s tracking");
    r.qp.max_iter = nuway_common::DeclareParam<int>(
        this, "qp.max_iter", r.qp.max_iter, "OSQP iteration budget");
    r.qp.rho = d("qp.rho", r.qp.rho, "OSQP rho");
    r.qp.eps_abs = d("qp.eps_abs", r.qp.eps_abs, "OSQP absolute tolerance");
    r.qp.eps_rel = d("qp.eps_rel", r.qp.eps_rel, "OSQP relative tolerance");
    r.qp.slack_weight = d("qp.slack_weight", r.qp.slack_weight, "slack weight");
    r.static_speed_mps =
        d("qp.static_speed_mps", r.static_speed_mps, "static agent speed");
    r.jerk_max_mps3 = model->limits.jerk_max_mps3;
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
    behaviors_.clear();
    samples_.clear();
    fallback_.clear();
    planner_->Reset();
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
    builtin_interfaces::msg::Time stamp = nuway_common::TickStamp(k);
    DiagStatus status = DiagStatus::kOk;
    std::string message;
    nuway_msgs::msg::TrajectoryCandidates candidates;
    nuway_msgs::msg::Trajectory trajectory;
    {
      const nuway_common::ScopedTimer timer(&cycle_ms);
      const auto* pose = Find(poses_, k);
      const auto* agents = Find(agents_, k);
      const auto* behavior = Find(behaviors_, k);
      const bool use_fallback = barrier_.IsDegraded(kInputSamples);
      const auto* samples =
          use_fallback ? Find(fallback_, k) : Find(samples_, k);
      if (pose != nullptr) {
        stamp = (*pose)->header.stamp;
      }
      std::string degraded;
      if (agents == nullptr) {
        degraded += " agents";
      }
      if (behavior == nullptr) {
        degraded += " behavior";
      }
      if (samples == nullptr) {
        degraded += use_fallback ? " fallback_samples" : " samples";
      } else if (use_fallback) {
        degraded += " samples(using fallback)";
      }
      if (pose == nullptr || !(*pose)->valid || route_ == nullptr) {
        // Case 3: nothing to stop in.
        if (pose == nullptr) {
          status = DiagStatus::kWarn;
          message = "pose missing (degraded)";
        } else if (route_ == nullptr) {
          message = "no route line";
        } else {
          message = "pose invalid";
        }
        const nuway_common::SE2 at =
            pose == nullptr ? nuway_common::SE2{}
                            : nuway_common::SE2FromMsg((*pose)->pose);
        trajectory = nuway_common::TrajectoryToMsg(
            nuway_common::StopTrajectory(at), stamp, "none", 0);
      } else {
        SceneInput in;
        in.ego = EgoObsFromMsg(**pose);
        in.route = route_.get();
        if (agents != nullptr) {
          in.agents = nuway_common::AgentStatesFromMsg(
              nuway_common::AgentsToMap(**agents, **pose));
        }
        if (samples != nullptr) {
          in.predictions = nuway_common::PredictionSetFromMsg(**samples);
        }
        std::optional<BehaviorOutput> decision;
        if (behavior != nullptr && agents != nullptr && samples != nullptr) {
          decision = BehaviorOutputFromMsg(**behavior);
        }
        const PlanResult result = planner_->Plan(in, decision);
        message = result.message;
        if (result.no_input) {
          status = DiagStatus::kWarn;
          trajectory = nuway_common::TrajectoryToMsg(
              nuway_common::StopTrajectory(in.ego.pose), stamp, "none", 0);
        } else {
          candidates = CandidatesToMsg(result, stamp);
          const Candidate* selected = result.selected();
          if (selected != nullptr) {
            trajectory = nuway_common::TrajectoryToMsg(
                selected->trajectory, stamp, selected->source, selected->id);
          } else {
            status = DiagStatus::kError;
            message += "; no selectable candidate";
            trajectory = nuway_common::TrajectoryToMsg(
                nuway_common::StopTrajectory(in.ego.pose), stamp, "none", 0);
          }
          if (result.qp_failed > 0) {
            status = DiagStatus::kWarn;
          }
        }
      }
      if (!degraded.empty()) {
        status = DiagStatus::kWarn;
        message += "; degraded:" + degraded;
      }
    }
    candidates.header.stamp = stamp;
    candidates.header.frame_id = nuway_common::kFrameMap;
    pub_candidates_->publish(candidates);
    pub_trajectory_->publish(trajectory);
    diag_.Publish(stamp, cycle_ms, 0.0, status, message);
  }

  // The scored set for the viz (docs/02 §4 TrajectoryCandidates).
  static nuway_msgs::msg::TrajectoryCandidates CandidatesToMsg(
      const PlanResult& result, const builtin_interfaces::msg::Time& stamp) {
    nuway_msgs::msg::TrajectoryCandidates msg;
    for (const char* name : kCostTermNames) {
      msg.cost_breakdown_names.emplace_back(name);
    }
    for (const Candidate& c : result.candidates) {
      msg.candidates.push_back(
          nuway_common::TrajectoryToMsg(c.trajectory, stamp, c.source, c.id));
      msg.cost.push_back(static_cast<float>(c.cost));
      for (std::size_t i = 0; i < kNumCostTerms; ++i) {
        msg.cost_breakdown.push_back(static_cast<float>(
            i < c.cost_breakdown.size() ? c.cost_breakdown[i] : 0.0));
      }
    }
    msg.selected_index = result.selected_index;
    return msg;
  }

  std::unique_ptr<Planner> planner_;
  nuway_msgs::msg::Route::ConstSharedPtr route_msg_;
  nuway_msgs::msg::ReferenceLine::ConstSharedPtr line_msg_;
  std::unique_ptr<RouteLine> route_;

  nuway_common::TickBarrier barrier_;
  std::optional<std::uint32_t> episode_id_;
  std::int64_t episode_start_k_ = 0;
  std::int64_t last_published_k_ = -1;
  std::map<std::int64_t, nuway_msgs::msg::EgoState::ConstSharedPtr> poses_;
  std::map<std::int64_t, nuway_msgs::msg::AgentArray::ConstSharedPtr> agents_;
  std::map<std::int64_t, nuway_msgs::msg::BehaviorDecision::ConstSharedPtr>
      behaviors_;
  std::map<std::int64_t, nuway_msgs::msg::PredictionSamples::ConstSharedPtr>
      samples_;
  std::map<std::int64_t, nuway_msgs::msg::PredictionSamples::ConstSharedPtr>
      fallback_;

  nuway_common::DiagPublisher diag_;
  rclcpp::Publisher<nuway_msgs::msg::TrajectoryCandidates>::SharedPtr
      pub_candidates_;
  rclcpp::Publisher<nuway_msgs::msg::Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Subscription<nuway_msgs::msg::Route>::SharedPtr sub_route_;
  rclcpp::Subscription<nuway_msgs::msg::ReferenceLine>::SharedPtr
      sub_reference_line_;
  rclcpp::Subscription<nuway_msgs::msg::EgoState>::SharedPtr sub_pose_;
  rclcpp::Subscription<nuway_msgs::msg::AgentArray>::SharedPtr sub_agents_;
  rclcpp::Subscription<nuway_msgs::msg::BehaviorDecision>::SharedPtr
      sub_behavior_;
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
  return nuway_common::RunNode<nuway_planning::PlannerNode>(argc, argv);
}
