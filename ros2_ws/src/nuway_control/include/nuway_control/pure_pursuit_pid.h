// Pure pursuit + PID controller on the reference line (M0 §2.7). Pure library:
// the node feeds it the ego pose (base_link = rear axle, docs/02 §1) and the
// body-frame speed once per tick and publishes what comes back.
//
// Lateral: lookahead L_d = clamp(k_v v + L_0, min, max) along the line from
// the ego projection; delta = atan(2 L sin(alpha) / L_d) with L the effective
// wheelbase of the vehicle model, clamped to max_steer_angle and rate-limited.
// Longitudinal: target speed = min over [s, s + horizon] of the curvature
// speed sqrt(a_lat_max / |kappa|) raised by the braking distance to it
// (sqrt(v_curv^2 + 2 a_plan (s' - s)), so a bend is slowed for early
// enough), bounded by speed_limit(s) and a stop profile toward the line end;
// PID on the speed error with clamped integral, output clamped to the vehicle
// limits and to what the LongitudinalMap can deliver at the current speed.
#ifndef NUWAY_CONTROL_PURE_PURSUIT_PID_H_
#define NUWAY_CONTROL_PURE_PURSUIT_PID_H_

#include <optional>
#include <vector>

#include <nuway_common/frenet.h>
#include <nuway_common/geometry.h>

#include "nuway_control/vehicle_model.h"

namespace nuway_control {

struct PurePursuitPidOptions {
  double lookahead_gain_s = 0.45;  // k_v
  double lookahead_base_m = 1.0;   // L_0
  double lookahead_min_m = 2.5;
  double lookahead_max_m = 20.0;
  double a_lat_max_mps2 = 1.5;
  // Floor of the speed-profile horizon; the profile looks at least
  // v_limit^2 / (2 plan_decel) ahead, the distance beyond which no bound can
  // cap the speed below the current limit (so the profile is continuous).
  double curvature_horizon_m = 60.0;
  double plan_decel_mps2 = 2.0;  // braking assumed when approaching a bend
  // Projection window around the previous s (a route that crosses or loops
  // back near itself must not pull the ego onto the other leg).
  double projection_back_m = 10.0;
  double projection_ahead_m = 50.0;
  double kp = 0.8;
  double ki = 0.2;
  double kd = 0.05;
  double integral_limit_mps2 = 0.5;  // |ki * integral| bound (anti-windup)
  double end_decel_mps2 = 1.5;       // stop profile toward the line end
  double max_lateral_error_m = 5.0;  // farther off the line: emergency stop
};

struct ControlOutput {
  double accel_mps2 = 0.0;
  double steering_angle_rad = 0.0;
  bool emergency_stop = true;
  double lateral_error_m = 0.0;
  double heading_error_rad = 0.0;
  double speed_error_mps = 0.0;
  double lookahead_m = 0.0;
  double target_speed_mps = 0.0;
  double s_m = 0.0;  // ego projection on the line
};

class PurePursuitPid {
 public:
  PurePursuitPid(VehicleModel model, PurePursuitPidOptions options);

  // Replaces the line; `speed_limit_mps` has one entry per line sample.
  // Keeps the PID state (a replan mid-episode is not a reset).
  void SetReferenceLine(nuway_common::ReferenceLine line,
                        std::vector<double> speed_limit_mps);
  bool has_reference_line() const { return has_line_; }
  // Drops the line and every cross-tick state (ResetEvent).
  void Reset();

  // One control step; `dt_s` is the time since the previous step (the tick
  // spacing). Returns emergency_stop when there is no line or the ego is off
  // it, and a stop command at the end of the line.
  ControlOutput Step(const nuway_common::SE2& pose, double speed_mps,
                     double dt_s);

  const PurePursuitPidOptions& options() const { return options_; }
  const VehicleModel& model() const { return model_; }

 private:
  double SpeedLimitAt(double s) const;
  double TargetSpeed(double s) const;
  double LateralStep(const nuway_common::SE2& pose, double speed_mps, double s,
                     double dt_s, ControlOutput* out);
  // Acceleration of the speed profile itself at `s` (feed-forward).
  double ProfileAccel(double s, double dt_s) const;
  double LongitudinalStep(double speed_mps, double target_mps,
                          double feedforward_mps2, double dt_s);

  VehicleModel model_;
  PurePursuitPidOptions options_;
  nuway_common::ReferenceLine line_;
  bool has_line_ = false;
  std::vector<double> speed_limit_mps_;
  // Drops the cross-tick state of the loops (integral, derivative memory,
  // steer rate limiter): after an emergency-stop tick the adapter sent
  // steer 0 / brake 1, so resuming from the old values would jump.
  void ResetTransients();

  double integral_ = 0.0;
  std::optional<double> prev_speed_mps_;  // derivative on the measurement
  double prev_steer_rad_ = 0.0;
  std::optional<double> last_s_m_;  // projection hint (previous tick's s)
};

}  // namespace nuway_control

#endif  // NUWAY_CONTROL_PURE_PURSUIT_PID_H_
