// safety_layer_node (M1 §3.7): the ROS shell around SafetyLayer, run on
// every tick. On a planning tick k it waits for the pose, agents,
// occupancy, trajectory and both prediction topics stamped k; on a
// control-only tick it waits for the pose alone and re-checks the last
// planning tick's trajectory, agents, predictions and grid against it
// (docs/02 §2). The output is always stamped with the tick it runs on
// and re-timed to it. A degraded planner (TickTimeout with its output
// missing) switches the layer to its held fallback for the episode; a
// missing or invalid pose publishes the no-input stop at the last pose.
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
#include <nuway_msgs/msg/ego_state.hpp>
#include <nuway_msgs/msg/occupancy_grid_mc.hpp>
#include <nuway_msgs/msg/prediction_samples.hpp>
#include <nuway_msgs/msg/reset_event.hpp>
#include <nuway_msgs/msg/tick_timeout.hpp>
#include <nuway_msgs/msg/trajectory.hpp>

#include "nuway_planning/msg_conv.h"
#include "nuway_planning/names.h"
#include "nuway_planning/safety_layer.h"

namespace nuway_planning {
namespace {

using nuway_common::DiagStatus;

constexpr const char* kInputPose = "pose";
constexpr const char* kInputAgents = "agents";
constexpr const char* kInputOccupancy = "occupancy";
constexpr const char* kInputTrajectory = "trajectory";
constexpr const char* kInputSamples = "samples";
constexpr const char* kInputFallback = "fallback_samples";
constexpr std::int64_t kBufferTicks = 4;

class SafetyLayerNode final : public rclcpp::Node {
 public:
  SafetyLayerNode()
      : rclcpp::Node(kSafetyLayerNodeName),
        barrier_({kInputPose, kInputAgents, kInputOccupancy, kInputTrajectory,
                  kInputSamples, kInputFallback}),
        diag_(this) {
    std::optional<SafetyOptions> options = DeclareOptions();
    if (!options.has_value()) {
      rclcpp::shutdown();
      return;
    }
    layer_ = std::make_unique<SafetyLayer>(*options);
    pub_safe_ = create_publisher<nuway_msgs::msg::Trajectory>(
        nuway_common::kTopicPlanningSafeTrajectory,
        nuway_common::qos::Stream());
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
    sub_occupancy_ = create_subscription<nuway_msgs::msg::OccupancyGridMC>(
        nuway_common::kTopicPerceptionOccupancy, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::OccupancyGridMC::ConstSharedPtr msg) {
          OnInput(kInputOccupancy, &grids_, std::move(msg));
        });
    sub_trajectory_ = create_subscription<nuway_msgs::msg::Trajectory>(
        nuway_common::kTopicPlanningTrajectory, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::Trajectory::ConstSharedPtr msg) {
          OnInput(kInputTrajectory, &trajectories_, std::move(msg));
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
  std::optional<SafetyOptions> DeclareOptions() {
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
    SafetyOptions o = SafetyOptions::FromVehicleModel(*model);
    o.collision.margin_lon_m =
        d("margin_lon_m", o.collision.margin_lon_m, "ego lon margin");
    o.collision.margin_lat_m =
        d("margin_lat_m", o.collision.margin_lat_m, "ego lat margin");
    o.collision.agent_margin_m =
        d("agent_margin_m", o.collision.agent_margin_m, "agent margin");
    o.collision_horizon_s =
        d("collision_horizon_s", o.collision_horizon_s, "check horizon");
    o.occupancy_threshold =
        d("occupancy_threshold", o.occupancy_threshold, "occupied threshold");
    o.occupancy_horizon_s =
        d("occupancy_horizon_s", o.occupancy_horizon_s, "occupancy horizon");
    o.a_gentle_mps2 = d("a_gentle_mps2", o.a_gentle_mps2, "gentle stop");
    return o;
  }

  template <typename MsgT>
  static void Prune(std::map<std::int64_t, MsgT>* buffer, std::int64_t k) {
    while (!buffer->empty() && buffer->begin()->first < k - kBufferTicks) {
      buffer->erase(buffer->begin());
    }
  }

  template <typename MsgT>
  static const MsgT* Find(const std::map<std::int64_t, MsgT>& buffer,
                          std::int64_t k) {
    const auto it = buffer.find(k);
    return it == buffer.end() ? nullptr : &it->second;
  }

  bool Accepts(std::int64_t k) const { return k >= episode_start_k_; }

  // Whether tick k has everything it waits for: the six inputs on a
  // planning tick, the pose alone on a control-only tick (the planning
  // inputs of k - 1 were settled before the world manager ticked k).
  bool Ready(std::int64_t k) const {
    if (nuway_common::IsPlanningTick(k)) {
      return barrier_.IsComplete(k);
    }
    return poses_.count(k) > 0;
  }

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
    if (Ready(k)) {
      Run(k);
    }
  }

  void OnResetEvent(const nuway_msgs::msg::ResetEvent& msg) {
    if (episode_id_.has_value() && msg.episode_id <= *episode_id_) {
      return;
    }
    episode_id_ = msg.episode_id;
    episode_start_k_ = nuway_common::TickIndex(msg.header.stamp);
    last_published_k_ = episode_start_k_ - 1;
    barrier_.Reset();
    poses_.clear();
    agents_.clear();
    grids_.clear();
    trajectories_.clear();
    samples_.clear();
    fallback_.clear();
    layer_->Reset();
    last_pose_.reset();
    RCLCPP_INFO(get_logger(), "reset: episode %u", msg.episode_id);
  }

  void OnTickTimeout(const nuway_msgs::msg::TickTimeout& msg) {
    const std::int64_t k = nuway_common::TickIndex(msg.header.stamp);
    if (!Accepts(k)) {
      return;
    }
    if (nuway_common::IsPlanningTick(k)) {
      for (const std::string& name : barrier_.Missing(k)) {
        barrier_.Degrade(name);
        RCLCPP_WARN(get_logger(), "tick %ld timed out; %s degraded until reset",
                    static_cast<long>(k), name.c_str());
      }
    } else if (poses_.count(k) == 0) {
      barrier_.Degrade(kInputPose);
      RCLCPP_WARN(get_logger(), "tick %ld timed out; pose degraded",
                  static_cast<long>(k));
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
    nuway_msgs::msg::Trajectory out_msg;
    {
      const nuway_common::ScopedTimer timer(&cycle_ms);
      const auto* pose = Find(poses_, k);
      if (pose != nullptr) {
        stamp = (*pose)->header.stamp;
      }
      if (pose == nullptr || !(*pose)->valid) {
        status = pose == nullptr ? DiagStatus::kWarn : DiagStatus::kOk;
        message = pose == nullptr ? "pose missing (degraded)" : "pose invalid";
        nuway_common::SE2 at;
        if (pose != nullptr) {
          at = nuway_common::SE2FromMsg((*pose)->pose);
        } else if (last_pose_.has_value()) {
          at = *last_pose_;
        }
        out_msg = nuway_common::TrajectoryToMsg(
            nuway_common::StopTrajectory(at), stamp, "none", 0);
      } else {
        // The planning tick whose outputs this tick reads.
        const std::int64_t kp = nuway_common::IsPlanningTick(k) ? k : k - 1;
        SafetyInput in;
        in.ego_pose = nuway_common::SE2FromMsg((*pose)->pose);
        in.ego_speed_mps = (*pose)->vx;
        last_pose_ = in.ego_pose;
        const auto* traj_msg = Find(trajectories_, kp);
        const auto* agents_msg = Find(agents_, kp);
        const auto* grid_msg = Find(grids_, kp);
        const bool use_fallback = barrier_.IsDegraded(kInputSamples);
        const auto* samples_msg =
            use_fallback ? Find(fallback_, kp) : Find(samples_, kp);
        const auto* pose_kp = Find(poses_, kp);
        nuway_common::Trajectory traj;
        std::vector<nuway_common::AgentState> agents;
        nuway_common::PredictionSet predictions;
        std::optional<OccupancyView> grid;
        if (traj_msg != nullptr) {
          traj = nuway_common::TrajectoryFromMsg(**traj_msg);
          in.trajectory = &traj;
          in.source = (*traj_msg)->source;
          in.trajectory_age_s =
              static_cast<double>(k - kp) * nuway_common::kTickDtS;
        } else {
          in.planner_degraded = true;
        }
        if (agents_msg != nullptr && pose_kp != nullptr) {
          agents = nuway_common::AgentStatesFromMsg(
              nuway_common::AgentsToMap(**agents_msg, **pose_kp));
        }
        in.agents = &agents;
        if (samples_msg != nullptr) {
          predictions = nuway_common::PredictionSetFromMsg(**samples_msg);
        }
        in.predictions = &predictions;
        if (grid_msg != nullptr && pose_kp != nullptr) {
          grid = OccupancyViewFromMsg(
              **grid_msg, nuway_common::SE2FromMsg((*pose_kp)->pose));
          if (grid.has_value()) {
            in.occupancy = &*grid;
          }
        }
        const SafetyOutput out = layer_->Step(in);
        out_msg = nuway_common::TrajectoryToMsg(
            out.trajectory, stamp, out.source,
            traj_msg != nullptr ? (*traj_msg)->candidate_id : 0);
        if (out.intervened) {
          status = DiagStatus::kWarn;
          message = "intervened: " + out.reason;
        } else {
          message = "pass";
        }
        std::string degraded;
        if (agents_msg == nullptr) {
          degraded += " agents";
        }
        if (grid_msg == nullptr) {
          degraded += " occupancy";
        }
        if (samples_msg == nullptr) {
          degraded += use_fallback ? " fallback_samples" : " samples";
        }
        if (!degraded.empty()) {
          status = DiagStatus::kWarn;
          message += "; missing:" + degraded;
        }
      }
    }
    pub_safe_->publish(out_msg);
    diag_.Publish(stamp, cycle_ms, 0.0, status, message);
  }

  std::unique_ptr<SafetyLayer> layer_;
  nuway_common::TickBarrier barrier_;
  std::optional<std::uint32_t> episode_id_;
  std::int64_t episode_start_k_ = 0;
  std::int64_t last_published_k_ = -1;
  std::optional<nuway_common::SE2> last_pose_;
  std::map<std::int64_t, nuway_msgs::msg::EgoState::ConstSharedPtr> poses_;
  std::map<std::int64_t, nuway_msgs::msg::AgentArray::ConstSharedPtr> agents_;
  std::map<std::int64_t, nuway_msgs::msg::OccupancyGridMC::ConstSharedPtr>
      grids_;
  std::map<std::int64_t, nuway_msgs::msg::Trajectory::ConstSharedPtr>
      trajectories_;
  std::map<std::int64_t, nuway_msgs::msg::PredictionSamples::ConstSharedPtr>
      samples_;
  std::map<std::int64_t, nuway_msgs::msg::PredictionSamples::ConstSharedPtr>
      fallback_;

  nuway_common::DiagPublisher diag_;
  rclcpp::Publisher<nuway_msgs::msg::Trajectory>::SharedPtr pub_safe_;
  rclcpp::Subscription<nuway_msgs::msg::EgoState>::SharedPtr sub_pose_;
  rclcpp::Subscription<nuway_msgs::msg::AgentArray>::SharedPtr sub_agents_;
  rclcpp::Subscription<nuway_msgs::msg::OccupancyGridMC>::SharedPtr
      sub_occupancy_;
  rclcpp::Subscription<nuway_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;
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
  return nuway_common::RunNode<nuway_planning::SafetyLayerNode>(argc, argv);
}
