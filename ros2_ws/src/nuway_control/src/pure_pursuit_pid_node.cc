// pure_pursuit_pid_node (M0 §2.7): runs once per tick, triggered by
// /nuway/loc/pose, and publishes /nuway/control/command stamped with that
// tick plus /nuway/control/debug. Follows the latched
// /nuway/route/reference_line directly (M0 has no planner in between).
//
// No-input convention (docs/02 §2): a pose with valid == false, no reference
// line for this episode yet, or an ego off the line still yields a command,
// with emergency_stop: true, so the lockstep gate never starves.
// Cross-tick state (PID integral, steer rate limiter, the line) is dropped on
// /nuway/sim/reset_event; poses and lines stamped before the reset are ignored
// (docs/02 §7). Single per-tick input, so no barrier; a TickTimeout for a
// tick whose pose never came is answered with an emergency stop stamped with
// that tick (the no-input output), so one lost pose does not time out every
// following tick.
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <nuway_common/diag.h>
#include <nuway_common/frames.h>
#include <nuway_common/frenet.h>
#include <nuway_common/geometry.h>
#include <nuway_common/node_main.h>
#include <nuway_common/params.h>
#include <nuway_common/qos.h>
#include <nuway_common/ros_conv.h>
#include <nuway_common/tick.h>
#include <nuway_msgs/msg/control_command.hpp>
#include <nuway_msgs/msg/control_debug.hpp>
#include <nuway_msgs/msg/ego_state.hpp>
#include <nuway_msgs/msg/reference_line.hpp>
#include <nuway_msgs/msg/reset_event.hpp>
#include <nuway_msgs/msg/tick_timeout.hpp>

#include "nuway_control/names.h"
#include "nuway_control/pure_pursuit_pid.h"
#include "nuway_control/vehicle_model.h"

namespace nuway_control {
namespace {

using nuway_common::DiagStatus;

class PurePursuitPidNode final : public rclcpp::Node {
 public:
  PurePursuitPidNode() : rclcpp::Node(kNodeName), diag_(this) {
    const auto vehicle_path = nuway_common::DeclareParam<std::string>(
        this, "vehicle", "configs/vehicle/lincoln_mkz_2020.yaml",
        "vehicle model YAML");
    PurePursuitPidOptions options;
    options.lookahead_gain_s = nuway_common::DeclareParam<double>(
        this, "lookahead_gain_s", options.lookahead_gain_s,
        "lookahead gain k_v");
    options.lookahead_base_m = nuway_common::DeclareParam<double>(
        this, "lookahead_base_m", options.lookahead_base_m, "lookahead L_0");
    options.lookahead_min_m = nuway_common::DeclareParam<double>(
        this, "lookahead_min_m", options.lookahead_min_m, "lookahead floor");
    options.lookahead_max_m = nuway_common::DeclareParam<double>(
        this, "lookahead_max_m", options.lookahead_max_m, "lookahead cap");
    options.a_lat_max_mps2 = nuway_common::DeclareParam<double>(
        this, "a_lat_max_mps2", options.a_lat_max_mps2,
        "lateral accel bound for the curvature speed");
    options.curvature_horizon_m = nuway_common::DeclareParam<double>(
        this, "curvature_horizon_m", options.curvature_horizon_m,
        "curvature lookahead for the speed bound");
    options.plan_decel_mps2 = nuway_common::DeclareParam<double>(
        this, "plan_decel_mps2", options.plan_decel_mps2,
        "braking assumed when approaching a bend");
    options.projection_back_m = nuway_common::DeclareParam<double>(
        this, "projection_back_m", options.projection_back_m,
        "Frenet projection window behind the previous s");
    options.projection_ahead_m = nuway_common::DeclareParam<double>(
        this, "projection_ahead_m", options.projection_ahead_m,
        "Frenet projection window ahead of the previous s");
    options.kp = nuway_common::DeclareParam<double>(this, "kp", options.kp,
                                                    "speed PID P gain");
    options.ki = nuway_common::DeclareParam<double>(this, "ki", options.ki,
                                                    "speed PID I gain");
    options.kd = nuway_common::DeclareParam<double>(this, "kd", options.kd,
                                                    "speed PID D gain");
    options.integral_limit_mps2 = nuway_common::DeclareParam<double>(
        this, "integral_limit_mps2", options.integral_limit_mps2,
        "anti-windup bound on the integral term");
    options.end_decel_mps2 = nuway_common::DeclareParam<double>(
        this, "end_decel_mps2", options.end_decel_mps2,
        "stop profile deceleration at the line end");
    options.max_lateral_error_m = nuway_common::DeclareParam<double>(
        this, "max_lateral_error_m", options.max_lateral_error_m,
        "off-line distance that triggers emergency_stop");

    std::string error;
    std::optional<VehicleModel> model = LoadVehicleModel(vehicle_path, &error);
    if (!model.has_value()) {
      // A controller without a vehicle model would silently stop the car on
      // every tick; failing loudly at start-up is the better outcome.
      RCLCPP_FATAL(get_logger(), "%s", error.c_str());
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(),
                "vehicle %s: wheelbase %.3f m (fitted %.3f), max steer %.3f "
                "rad, a in [%.1f, %.1f]",
                model->name.c_str(), model->wheelbase_m,
                model->wheelbase_fitted_m, model->max_steer_angle_rad,
                model->limits.a_min_mps2, model->limits.a_max_mps2);
    controller_ = std::make_unique<PurePursuitPid>(std::move(*model), options);

    pub_command_ = create_publisher<nuway_msgs::msg::ControlCommand>(
        nuway_common::kTopicControlCommand, nuway_common::qos::Stream());
    pub_debug_ = create_publisher<nuway_msgs::msg::ControlDebug>(
        nuway_common::kTopicControlDebug, nuway_common::qos::Stream());
    sub_reference_line_ = create_subscription<nuway_msgs::msg::ReferenceLine>(
        nuway_common::kTopicReferenceLine, nuway_common::qos::Latched(),
        [this](const nuway_msgs::msg::ReferenceLine& msg) {
          OnReferenceLine(msg);
        });
    sub_pose_ = create_subscription<nuway_msgs::msg::EgoState>(
        nuway_common::kTopicPose, nuway_common::qos::Stream(),
        [this](const nuway_msgs::msg::EgoState& msg) { OnPose(msg); });
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
  // Stamped before the current episode's first tick (docs/02 §7). Compared
  // as tick indices, never as raw stamps: the ResetEvent and the first pose
  // of an episode are stamped by different float paths and may differ by a
  // few nanoseconds.
  bool BeforeEpisode(const builtin_interfaces::msg::Time& stamp) const {
    return episode_tick_.has_value() &&
           nuway_common::TickIndex(stamp) < *episode_tick_ &&
           rclcpp::Time(stamp).nanoseconds() != 0;
  }

  void OnResetEvent(const nuway_msgs::msg::ResetEvent& msg) {
    // The event topic is transient_local: a (re)started controller receives
    // earlier episodes' events too, in no guaranteed order relative to the
    // latched line. Episode ids only grow, so anything not newer is a replay
    // and must not wipe the line of the episode we are already in.
    if (episode_id_.has_value() && msg.episode_id <= *episode_id_) {
      return;
    }
    episode_id_ = msg.episode_id;
    episode_tick_ = nuway_common::TickIndex(msg.header.stamp);
    controller_->Reset();
    last_tick_.reset();
    RCLCPP_INFO(get_logger(), "reset: episode %u", msg.episode_id);
  }

  void OnReferenceLine(const nuway_msgs::msg::ReferenceLine& msg) {
    if (BeforeEpisode(msg.header.stamp)) {
      RCLCPP_WARN(get_logger(),
                  "reference line stamped before the current episode; ignored");
      return;
    }
    const std::size_t n = msg.points.size();
    if (n < 2 || msg.s.size() != n || msg.heading.size() != n ||
        msg.curvature.size() != n) {
      // FromSamples trusts the sizes; a malformed message would index out
      // of bounds on every tick.
      RCLCPP_ERROR(get_logger(),
                   "reference line with %zu points but %zu s / %zu heading / "
                   "%zu curvature samples; ignored",
                   n, msg.s.size(), msg.heading.size(), msg.curvature.size());
      return;
    }
    nuway_common::Vector2dList points;
    points.reserve(n);
    for (const geometry_msgs::msg::Point& p : msg.points) {
      points.emplace_back(p.x, p.y);
    }
    const std::vector<double> s(msg.s.begin(), msg.s.end());
    const std::vector<double> heading(msg.heading.begin(), msg.heading.end());
    const std::vector<double> curvature(msg.curvature.begin(),
                                        msg.curvature.end());
    std::vector<double> speed_limit(msg.speed_limit.begin(),
                                    msg.speed_limit.end());
    controller_->SetReferenceLine(
        nuway_common::ReferenceLine::FromSamples(points, s, heading, curvature),
        std::move(speed_limit));
    RCLCPP_INFO(get_logger(), "reference line: %zu points, %.0f m", n,
                s.back());
  }

  void OnPose(const nuway_msgs::msg::EgoState& msg) {
    if (BeforeEpisode(msg.header.stamp)) {
      return;  // previous episode; world_manager is not waiting on it
    }
    const std::int64_t tick = nuway_common::TickIndex(msg.header.stamp);
    if (last_tick_.has_value() && tick <= *last_tick_) {
      return;  // already answered (a late pose after a TickTimeout e-stop)
    }
    double cycle_ms = 0.0;
    ControlOutput out;
    {
      const nuway_common::ScopedTimer timer(&cycle_ms);
      const double dt_s = last_tick_.has_value()
                              ? static_cast<double>(std::max<std::int64_t>(
                                    1, tick - *last_tick_)) *
                                    nuway_common::kTickDtS
                              : nuway_common::kTickDtS;
      last_tick_ = tick;
      if (msg.valid) {
        out =
            controller_->Step(nuway_common::SE2FromMsg(msg.pose), msg.vx, dt_s);
      }
    }
    std::string message;
    DiagStatus status = DiagStatus::kOk;
    if (!msg.valid) {
      message = "pose invalid";
    } else if (!controller_->has_reference_line()) {
      message = "no reference line";
    } else if (out.emergency_stop) {
      message = "off the line or past its end";
      status = DiagStatus::kWarn;
    }
    Publish(msg.header.stamp, out, cycle_ms, status, message);
  }

  // The no-input convention for the controller's one per-tick input
  // (docs/02 §2): when the pose of tick k never came, the gate's timeout is
  // answered with an emergency stop stamped k instead of leaving every later
  // tick to time out as well.
  void OnTickTimeout(const nuway_msgs::msg::TickTimeout& msg) {
    if (BeforeEpisode(msg.header.stamp)) {
      return;
    }
    const std::int64_t tick = nuway_common::TickIndex(msg.header.stamp);
    if (last_tick_.has_value() && tick <= *last_tick_) {
      return;  // the pose did arrive and was answered
    }
    last_tick_ = tick;
    RCLCPP_WARN(get_logger(), "tick %ld timed out without a pose; e-stop",
                static_cast<long>(tick));
    Publish(nuway_common::TickStamp(tick), ControlOutput{}, 0.0,
            DiagStatus::kWarn, "no pose for the tick (timeout)");
  }

  void Publish(const builtin_interfaces::msg::Time& stamp,
               const ControlOutput& out, double cycle_ms, DiagStatus status,
               const std::string& message) {
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
    debug.lookahead = static_cast<float>(out.lookahead_m);
    debug.solve_time_ms = static_cast<float>(cycle_ms);
    debug.solver_ok = !out.emergency_stop;
    pub_debug_->publish(debug);
    diag_.Publish(stamp, cycle_ms, 0.0, status, message);
  }

  std::unique_ptr<PurePursuitPid> controller_;
  std::optional<std::uint32_t> episode_id_;
  std::optional<std::int64_t> episode_tick_;  // first tick of the episode
  std::optional<std::int64_t> last_tick_;     // last tick answered

  nuway_common::DiagPublisher diag_;
  rclcpp::Publisher<nuway_msgs::msg::ControlCommand>::SharedPtr pub_command_;
  rclcpp::Publisher<nuway_msgs::msg::ControlDebug>::SharedPtr pub_debug_;
  rclcpp::Subscription<nuway_msgs::msg::ReferenceLine>::SharedPtr
      sub_reference_line_;
  rclcpp::Subscription<nuway_msgs::msg::EgoState>::SharedPtr sub_pose_;
  rclcpp::Subscription<nuway_msgs::msg::ResetEvent>::SharedPtr sub_reset_;
  rclcpp::Subscription<nuway_msgs::msg::TickTimeout>::SharedPtr
      sub_tick_timeout_;
};

}  // namespace
}  // namespace nuway_control

int main(int argc, char** argv) {
  return nuway_common::RunNode<nuway_control::PurePursuitPidNode>(argc, argv);
}
