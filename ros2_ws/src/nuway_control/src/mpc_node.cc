// mpc_node (M1 §3.8): the ROS shell around Mpc, the only producer of
// /nuway/control/command from M1 on. Runs once per tick after the pose and
// the safe trajectory stamped with that tick have both arrived (the
// current-tick barrier of docs/02 §2), stamps the ControlCommand with that
// tick and publishes ControlDebug, the predicted horizon and a NodeDiag
// alongside. The lockstep gate waits for the command, so a slow solve
// costs wall clock, never determinism.
//
// No-input convention: a valid: false pose, or a TickTimeout that found the
// pose or the safe trajectory missing (which degrades that input until the
// next ResetEvent), publishes emergency_stop: true stamped with the tick,
// with the steering angle at the delta_ref last tracked (§3.8). A
// source: "none" safe trajectory is tracked like any other and holds the
// car still. Cross-tick state (the warm start, the command history, the
// failure counter, the degraded flags) is dropped on ResetEvent; messages
// stamped before the current episode are ignored, compared as tick
// indices.
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <nuway_common/diag.h>
#include <nuway_common/frames.h>
#include <nuway_common/geometry.h>
#include <nuway_common/node_main.h>
#include <nuway_common/params.h>
#include <nuway_common/qos.h>
#include <nuway_common/ros_conv.h>
#include <nuway_common/tick.h>
#include <nuway_common/trajectory.h>
#include <nuway_msgs/msg/control_command.hpp>
#include <nuway_msgs/msg/control_debug.hpp>
#include <nuway_msgs/msg/ego_state.hpp>
#include <nuway_msgs/msg/reset_event.hpp>
#include <nuway_msgs/msg/tick_timeout.hpp>
#include <nuway_msgs/msg/trajectory.hpp>

#include "nuway_control/mpc.h"
#include "nuway_control/names.h"
#include "nuway_control/vehicle_model.h"

namespace nuway_control {
namespace {

using nuway_common::DiagStatus;

constexpr const char* kInputPose = "pose";
constexpr const char* kInputTrajectory = "safe_trajectory";
constexpr std::int64_t kBufferTicks = 4;

class MpcNode final : public rclcpp::Node {
 public:
  MpcNode()
      : rclcpp::Node(kMpcNodeName),
        barrier_({kInputPose, kInputTrajectory}),
        diag_(this) {
    if (!Configure()) {
      rclcpp::shutdown();
      return;
    }
    pub_command_ = create_publisher<nuway_msgs::msg::ControlCommand>(
        nuway_common::kTopicControlCommand, nuway_common::qos::Stream());
    pub_debug_ = create_publisher<nuway_msgs::msg::ControlDebug>(
        nuway_common::kTopicControlDebug, nuway_common::qos::Stream());
    pub_horizon_ = create_publisher<nuway_msgs::msg::Trajectory>(
        nuway_common::kTopicControlHorizon, nuway_common::qos::Stream());
    sub_pose_ = create_subscription<nuway_msgs::msg::EgoState>(
        nuway_common::kTopicPose, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::EgoState::ConstSharedPtr msg) {
          OnInput(kInputPose, &poses_, std::move(msg));
        });
    sub_trajectory_ = create_subscription<nuway_msgs::msg::Trajectory>(
        nuway_common::kTopicPlanningSafeTrajectory, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::Trajectory::ConstSharedPtr msg) {
          OnInput(kInputTrajectory, &trajectories_, std::move(msg));
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
  // Declares the parameters (defaults mirror MpcOptions and
  // config/defaults.yaml), loads the vehicle model and builds the
  // controller; false when the model cannot be loaded.
  bool Configure() {
    const auto d = [this](const char* name, double value, const char* doc) {
      return nuway_common::DeclareParam<double>(this, name, value, doc);
    };
    const auto i = [this](const char* name, int value, const char* doc) {
      return nuway_common::DeclareParam<int>(this, name, value, doc);
    };
    const auto vehicle_path = nuway_common::DeclareParam<std::string>(
        this, "vehicle", "configs/vehicle/lincoln_mkz_2020.yaml",
        "vehicle model YAML");
    MpcOptions o;
    o.horizon = i("horizon", o.horizon, "N steps of one tick");
    o.t_delay_s = d("t_delay_s", o.t_delay_s, "actuation delay compensated");
    o.q[0] = d("q.lon", o.q[0], "longitudinal position error weight");
    o.q[1] = d("q.lat", o.q[1], "lateral position error weight");
    o.q[2] = d("q.yaw", o.q[2], "heading error weight");
    o.q[3] = d("q.v", o.q[3], "speed error weight");
    o.q[4] = d("q.steer", o.q[4], "wheel angle error weight");
    o.q_terminal_scale =
        d("q_terminal_scale", o.q_terminal_scale, "last knot's Q multiplier");
    o.r[0] = d("r.accel", o.r[0], "accel deviation weight");
    o.r[1] = d("r.steer", o.r[1], "steer deviation weight");
    o.r_d[0] = d("r_d.accel", o.r_d[0], "accel rate weight");
    o.r_d[1] = d("r_d.steer", o.r_d[1], "steer rate weight");
    o.max_consecutive_solver_failures =
        i("max_consecutive_solver_failures", o.max_consecutive_solver_failures,
          "counted failures in a row before emergency_stop");
    o.accel_bias_tau_s = d("accel_bias_tau_s", o.accel_bias_tau_s,
                           "disturbance observer time constant (0: off)");
    o.accel_bias_max_mps2 = d("accel_bias_max_mps2", o.accel_bias_max_mps2,
                              "disturbance estimate bound");
    o.accel_bias_stop_speed_mps =
        d("accel_bias_stop_speed_mps", o.accel_bias_stop_speed_mps,
          "no observer update while braking below this speed");
    o.use_measured_steering = nuway_common::DeclareParam<bool>(
        this, "use_measured_steering", o.use_measured_steering,
        "wheel angle from the pose (true) or the lag model (false)");
    o.qp.max_iter = i("qp.max_iter", o.qp.max_iter, "fixed OSQP budget");
    o.qp.check_termination =
        i("qp.check_termination", o.qp.check_termination, "OSQP check period");
    o.qp.eps_abs = d("qp.eps_abs", o.qp.eps_abs, "OSQP absolute tolerance");
    o.qp.eps_rel = d("qp.eps_rel", o.qp.eps_rel, "OSQP relative tolerance");
    o.qp.rho = d("qp.rho", o.qp.rho, "fixed ADMM step");
    o.qp.polish = nuway_common::DeclareParam<bool>(this, "qp.polish",
                                                   o.qp.polish, "OSQP polish");
    std::string error;
    std::optional<VehicleModel> model = LoadVehicleModel(vehicle_path, &error);
    if (!model.has_value()) {
      // A controller without a vehicle model would stop the car on every
      // tick; failing loudly at start-up is the better outcome.
      RCLCPP_FATAL(get_logger(), "%s", error.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(),
                "vehicle %s: L %.3f m (fitted %.3f), tau_steer %.3f s, max "
                "steer %.3f rad; N %d, t_delay %.2f s, max_iter %d",
                model->name.c_str(), model->wheelbase_m,
                model->wheelbase_fitted_m, model->tau_steer_s,
                model->max_steer_angle_rad, o.horizon, o.t_delay_s,
                o.qp.max_iter);
    mpc_ = std::make_unique<Mpc>(std::move(*model), o);
    return true;
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

  template <typename MsgT>
  void OnInput(const char* name, std::map<std::int64_t, MsgT>* buffer,
               MsgT msg) {
    const std::int64_t k = nuway_common::TickIndex(msg->header.stamp);
    if (!Accepts(k)) {
      return;  // previous episode; world_manager is not waiting on it
    }
    (*buffer)[k] = std::move(msg);
    Prune(buffer, k);
    barrier_.Arrive(name, k);
    if (barrier_.IsComplete(k)) {
      Run(k);
    }
  }

  void OnResetEvent(const nuway_msgs::msg::ResetEvent& msg) {
    // The event topic is transient_local: a (re)started node receives
    // earlier episodes' events too. Episode ids only grow, so anything
    // not newer is a replay.
    if (episode_id_.has_value() && msg.episode_id <= *episode_id_) {
      return;
    }
    episode_id_ = msg.episode_id;
    episode_start_k_ = nuway_common::TickIndex(msg.header.stamp);
    last_published_k_ = episode_start_k_ - 1;
    barrier_.Reset();
    poses_.clear();
    trajectories_.clear();
    mpc_->Reset();
    RCLCPP_INFO(get_logger(), "reset: episode %u", msg.episode_id);
  }

  // The gate timed out on tick k: whatever is still missing is degraded
  // until the next reset (docs/02 §2) and the tick is answered now.
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
      return;  // already answered (a late input after a TickTimeout)
    }
    last_published_k_ = k;
    double cycle_ms = 0.0;
    builtin_interfaces::msg::Time stamp = nuway_common::TickStamp(k);
    MpcOutput out;
    std::string message;
    {
      const nuway_common::ScopedTimer timer(&cycle_ms);
      const auto* pose = Find(poses_, k);
      const auto* traj_msg = Find(trajectories_, k);
      MpcInput in;
      nuway_common::Trajectory traj;
      if (pose != nullptr) {
        stamp = (*pose)->header.stamp;
        in.pose = nuway_common::SE2FromMsg((*pose)->pose);
        in.speed_mps = (*pose)->vx;
        in.accel_mps2 = (*pose)->ax;
        in.steering_angle_rad = (*pose)->steering_angle;
        in.pose_valid = (*pose)->valid;
      } else {
        in.pose_valid = false;
      }
      if (traj_msg != nullptr) {
        traj = nuway_common::TrajectoryFromMsg(**traj_msg);
        in.trajectory = &traj;
      }
      in.trajectory_degraded = barrier_.IsDegraded(kInputTrajectory);
      out = mpc_->Step(in);
      if (pose == nullptr) {
        message = "pose missing (degraded)";
      } else if (!in.pose_valid) {
        message = "pose invalid";
      } else if (in.trajectory_degraded) {
        message = "safe_trajectory degraded";
      } else if (traj_msg == nullptr) {
        message = "safe_trajectory missing";
      } else if (out.emergency_stop) {
        message = "emergency stop: " + out.status;
      } else if (out.counted_failure) {
        message = "solver failure " + std::to_string(out.consecutive_failures) +
                  ": " + out.status;
      } else {
        message = out.status;
      }
    }
    Publish(stamp, out, cycle_ms, message);
  }

  // Emits the command, the debug telemetry, the predicted horizon and the
  // NodeDiag for one tick, all stamped with `stamp`.
  void Publish(const builtin_interfaces::msg::Time& stamp, const MpcOutput& out,
               double cycle_ms, const std::string& message) {
    nuway_msgs::msg::ControlCommand command;
    command.header.stamp = stamp;
    command.accel = static_cast<float>(out.accel_mps2);
    command.steering_angle = static_cast<float>(out.steering_angle_rad);
    command.emergency_stop = out.emergency_stop;
    pub_command_->publish(command);

    nuway_msgs::msg::ControlDebug debug;
    debug.header.stamp = stamp;
    debug.lateral_error = static_cast<float>(out.lateral_error_m);
    debug.heading_error = static_cast<float>(out.heading_error_rad);
    debug.speed_error = static_cast<float>(out.speed_error_mps);
    debug.lookahead = 0.0F;
    debug.solve_time_ms = static_cast<float>(out.solve_time_ms);
    debug.solver_ok = out.solver_ok;
    pub_debug_->publish(debug);

    pub_horizon_->publish(
        nuway_common::TrajectoryToMsg(out.horizon, stamp, "mpc", 0));
    const DiagStatus status = out.emergency_stop || out.counted_failure
                                  ? DiagStatus::kWarn
                                  : DiagStatus::kOk;
    diag_.Publish(stamp, cycle_ms, 0.0, status, message);
  }

  std::unique_ptr<Mpc> mpc_;
  nuway_common::TickBarrier barrier_;
  std::optional<std::uint32_t> episode_id_;
  std::int64_t episode_start_k_ = 0;
  std::int64_t last_published_k_ = -1;
  std::map<std::int64_t, nuway_msgs::msg::EgoState::ConstSharedPtr> poses_;
  std::map<std::int64_t, nuway_msgs::msg::Trajectory::ConstSharedPtr>
      trajectories_;

  nuway_common::DiagPublisher diag_;
  rclcpp::Publisher<nuway_msgs::msg::ControlCommand>::SharedPtr pub_command_;
  rclcpp::Publisher<nuway_msgs::msg::ControlDebug>::SharedPtr pub_debug_;
  rclcpp::Publisher<nuway_msgs::msg::Trajectory>::SharedPtr pub_horizon_;
  rclcpp::Subscription<nuway_msgs::msg::EgoState>::SharedPtr sub_pose_;
  rclcpp::Subscription<nuway_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<nuway_msgs::msg::ResetEvent>::SharedPtr sub_reset_;
  rclcpp::Subscription<nuway_msgs::msg::TickTimeout>::SharedPtr
      sub_tick_timeout_;
};

}  // namespace
}  // namespace nuway_control

// Standard single-threaded spin (nuway_common/node_main.h).
int main(int argc, char** argv) {
  return nuway_common::RunNode<nuway_control::MpcNode>(argc, argv);
}
