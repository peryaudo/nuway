// Pure pursuit + PID controller on the reference line (M0 §2.7). Pure library:
// the node feeds it the ego pose (base_link = rear axle, docs/02 §1) and the
// body-frame speed once per tick and publishes what comes back. It stays in
// the repo permanently as the simplest fallback and sanity-check controller.
//
// Lateral: pure pursuit. The kinematic bicycle model collapses the car to
// one steered front wheel and one fixed rear wheel a wheelbase L apart; at
// a front steer angle delta the rear axle moves on a circle of curvature
// kappa = tan(delta) / L, so any wanted curvature maps to a steer through
// delta = atan(kappa L). Pure pursuit chooses that curvature geometrically:
// take the point of the line a lookahead distance L_d ahead of the ego's
// projection and steer onto the circle that passes through both the rear
// axle and that point. With alpha the angle between the heading and the
// chord to the target, that circle has curvature 2 sin(alpha) / L_d (a
// chord of length L_d subtends 2 alpha at the centre: L_d = 2 R sin(alpha)),
// and so
//
//   delta = atan(2 L sin(alpha) / L_d).
//
// L_d = clamp(k_v v + L_0, min, max) grows with speed because the point the
// car steers toward is where it will be in about k_v seconds: a short
// lookahead tracks the line tightly at low speed but oscillates at high
// speed (the car overshoots a point it reaches within a tick or two), a
// long one is smooth but cuts corners; scaling with speed keeps the time to
// the target roughly constant. The formula's L_d is the straight chord to
// the target, so the chord is what this file uses, not the arc length along
// the line (they differ on curves and at the line end, where the target is
// continued straight past the last sample; Decisions log, task 10). L is
// the effective wheelbase L_fit + K v^2 of the vehicle model: the sysid
// steer sweeps (M0 §2.6) show the car turning less at speed than
// tan(delta) / L predicts (tyre slip, an understeer gradient K), and
// inflating L reproduces the measured yaw rate. The steer is clamped to
// max_steer_angle and rate-limited to steer_rate_max of the model, so the
// command never exceeds what the actuator was measured doing.
//
// Longitudinal: a speed profile along the line and a PID on the speed error.
// Target speed = min over [s, s + horizon] of the curvature speed
// sqrt(a_lat_max / |kappa|) (lateral acceleration v^2 kappa reaches
// a_lat_max there) raised by the braking distance to it
// (sqrt(v_curv^2 + 2 a_plan (s' - s)): the fastest the ego may go now and
// still be down to v_curv at s' when braking at a_plan, so a bend is slowed
// for early enough), bounded by speed_limit(s) and a stop profile
// sqrt(2 a_end remaining) toward the line end. The horizon must reach at
// least v_limit^2 / (2 a_plan) for the profile to be continuous (see
// PurePursuitPidOptions::curvature_horizon_m). The PID has its derivative
// on the measurement, a clamped and conditionally integrated integral
// (anti-windup), and the profile's own v dv/ds as feed-forward; the output
// is clamped to the vehicle limits and to what the LongitudinalMap can
// deliver at the current speed. Each term is explained at the function that
// computes it in pure_pursuit_pid.cc.
#ifndef NUWAY_CONTROL_PURE_PURSUIT_PID_H_
#define NUWAY_CONTROL_PURE_PURSUIT_PID_H_

#include <optional>
#include <vector>

#include <nuway_common/frenet.h>
#include <nuway_common/geometry.h>

#include "nuway_control/vehicle_model.h"

namespace nuway_control {

// Tuning knobs; the defaults are the task 13 tuning of M0 §2.7 and are
// mirrored by config/defaults.yaml (the node declares each as a parameter).
struct PurePursuitPidOptions {
  double lookahead_gain_s = 0.45;  // k_v: seconds of travel to the target
  double lookahead_base_m = 1.0;   // L_0: lookahead at standstill
  double lookahead_min_m = 2.5;
  double lookahead_max_m = 20.0;
  double a_lat_max_mps2 = 1.5;  // curvature speed = sqrt(a_lat_max / |kappa|)
  // Floor of the speed-profile horizon; the profile looks at least
  // v_limit^2 / (2 plan_decel) ahead, the distance beyond which no bound can
  // cap the speed below the current limit (so the profile is continuous).
  double curvature_horizon_m = 60.0;
  double plan_decel_mps2 = 2.0;  // braking assumed when approaching a bend
  // Projection window around the previous s (a route that crosses or loops
  // back near itself must not pull the ego onto the other leg).
  double projection_back_m = 10.0;
  double projection_ahead_m = 50.0;
  double kp = 0.8;                   // m/s^2 per m/s of speed error
  double ki = 0.2;                   // m/s^2 per m/s * s of accumulated error
  double kd = 0.05;                  // m/s^2 per m/s^2 of measured deceleration
  double integral_limit_mps2 = 0.5;  // |ki * integral| bound (anti-windup)
  double end_decel_mps2 = 1.5;       // stop profile toward the line end
  double max_lateral_error_m = 5.0;  // farther off the line: emergency stop
};

// One tick's result. The first three fields become the ControlCommand; the
// rest is the ControlDebug telemetry (docs/02 §4).
struct ControlOutput {
  double accel_mps2 = 0.0;          // longitudinal command (>= 0 throttle)
  double steering_angle_rad = 0.0;  // front wheel angle, ccw positive (ROS)
  bool emergency_stop = true;       // the no-input default: steer 0, brake 1
  double lateral_error_m = 0.0;     // signed Frenet d of the rear axle
  double heading_error_rad = 0.0;   // line heading - ego yaw, wrapped
  double speed_error_mps = 0.0;     // target - measured
  double lookahead_m = 0.0;         // L_d used this tick
  double target_speed_mps = 0.0;    // speed profile at s
  double s_m = 0.0;                 // ego projection on the line
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
  // Posted speed limit at arc length s (the lower of the bracketing samples).
  double SpeedLimitAt(double s) const;
  // The speed profile: limit, curvature bounds over the horizon, end stop.
  double TargetSpeed(double s) const;
  // Pure pursuit steer for this tick; also fills out->lookahead_m.
  double LateralStep(const nuway_common::SE2& pose, double speed_mps, double s,
                     double dt_s, ControlOutput* out);
  // Acceleration of the speed profile itself at `s` (feed-forward).
  double ProfileAccel(double s, double dt_s) const;
  // PID on the speed error plus feed-forward, clamped to what the vehicle
  // can do; updates the integral and derivative memory.
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

  double integral_ = 0.0;                 // accumulated speed error, m/s * s
  std::optional<double> prev_speed_mps_;  // derivative on the measurement
  double prev_steer_rad_ = 0.0;           // steer rate limiter memory
  std::optional<double> last_s_m_;        // projection hint (previous tick's s)
};

}  // namespace nuway_control

#endif  // NUWAY_CONTROL_PURE_PURSUIT_PID_H_
