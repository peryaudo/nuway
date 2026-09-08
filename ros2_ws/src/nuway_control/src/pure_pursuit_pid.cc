// PurePursuitPid: the per-tick arithmetic of the controller described in
// the header. Every function below states the formula it evaluates and why
// the term exists; the tuning history is in M0 §2.7 and the Decisions log
// (tasks 10 and 13, and the task 4 / 10 review fixes).
#include "nuway_control/pure_pursuit_pid.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace nuway_control {

PurePursuitPid::PurePursuitPid(VehicleModel model,
                               PurePursuitPidOptions options)
    : model_(std::move(model)), options_(options) {}

// Installs a new line. The speed limit vector is brought to one entry per
// sample (a short vector repeats its last value, an empty one means 0 m/s,
// i.e. the profile stops the car); the projection hint is dropped because
// arc length s is measured along the new line, but the PID and steer state
// survive: a replan mid-episode is not a reset and the car keeps moving.
void PurePursuitPid::SetReferenceLine(nuway_common::ReferenceLine line,
                                      std::vector<double> speed_limit_mps) {
  speed_limit_mps.resize(
      static_cast<std::size_t>(line.size()),
      speed_limit_mps.empty() ? 0.0 : speed_limit_mps.back());
  line_ = std::move(line);
  has_line_ = true;
  speed_limit_mps_ = std::move(speed_limit_mps);
  last_s_m_.reset();  // s is measured along the new line
}

// Back to the freshly constructed state (ResetEvent, docs/02 §7).
void PurePursuitPid::Reset() {
  line_ = nuway_common::ReferenceLine{};
  has_line_ = false;
  speed_limit_mps_.clear();
  ResetTransients();
  last_s_m_.reset();
}

// Zeroes the loop memories only; the line and the projection hint stay.
void PurePursuitPid::ResetTransients() {
  integral_ = 0.0;
  prev_speed_mps_.reset();
  prev_steer_rad_ = 0.0;
}

// Speed limit at s: the lower of the two samples bracketing s, so a limit
// drop at sample i+1 already applies over segment i (at most one sample
// spacing early; conservative, never late). 0 without a limit vector.
double PurePursuitPid::SpeedLimitAt(double s) const {
  if (speed_limit_mps_.empty()) {
    return 0.0;
  }
  const auto idx = static_cast<std::size_t>(line_.SegmentIndex(s));
  const std::size_t next = std::min(idx + 1, speed_limit_mps_.size() - 1);
  return std::min(speed_limit_mps_[idx], speed_limit_mps_[next]);
}

// The speed profile v*(s), the minimum of four bounds:
//   1. the posted limit at s;
//   2. for every sample s' in [s, s + horizon]: the speed the ego may have
//      now and still reach s' at v_there = min(limit(s'),
//      sqrt(a_lat_max / |kappa(s')|)) while braking at plan_decel, i.e.
//      sqrt(v_there^2 + 2 plan_decel (s' - s)) (v^2 = v_0^2 + 2 a d);
//   3. the curvature speed at s itself (s lies between samples);
//   4. the stop profile sqrt(2 end_decel remaining) toward the line end,
//      which reaches 0 exactly at the last sample.
// The curvature speed comes from a_lat = v^2 kappa <= a_lat_max. Bound 2 is
// what makes the profile a braking ramp rather than a cliff: it decreases
// smoothly as the ego approaches the bend, and its slope is exactly the
// deceleration the feed-forward (ProfileAccel) wants.
double PurePursuitPid::TargetSpeed(double s) const {
  double target = SpeedLimitAt(s);
  // Bounds over the horizon: a bend or a lower limit ahead caps the speed now
  // to what a plan_decel_mps2 braking reaches it with, so the profile is
  // continuous and its slope is a usable feed-forward. Continuity needs the
  // horizon to reach v_limit^2 / (2 a): a bound entering a shorter horizon
  // caps the speed in one step (24.6 -> 15.8 m/s at 60 m), and beyond that
  // distance sqrt(2 a ds) alone already exceeds the limit. With
  // plan_decel_mps2 <= 0 the ramp term vanishes and the fixed horizon is
  // used as is (a bound then applies at its full value on entry).
  const double v_limit = target;
  const double braking_distance =
      options_.plan_decel_mps2 > 0.0
          ? (v_limit * v_limit) / (2.0 * options_.plan_decel_mps2)
          : options_.curvature_horizon_m;
  const double horizon =
      std::max(options_.curvature_horizon_m, braking_distance);
  const std::vector<double>& line_s = line_.s();
  const double s_end = std::min(line_.length(), s + horizon);
  const auto first = std::lower_bound(line_s.begin(), line_s.end(), s);
  for (auto it = first; it != line_s.end() && *it <= s_end; ++it) {
    const std::size_t i = static_cast<std::size_t>(it - line_s.begin());
    const double kappa = std::abs(line_.curvature()[i]);
    double v_there = SpeedLimitAt(*it);
    if (kappa > 1e-6) {  // straight: no lateral bound (1e-6 = R of 1000 km)
      v_there = std::min(v_there, std::sqrt(options_.a_lat_max_mps2 / kappa));
    }
    const double v_here = std::sqrt(
        (v_there * v_there) + (2.0 * options_.plan_decel_mps2 * (*it - s)));
    target = std::min(target, v_here);
  }
  const double kappa_here = std::abs(line_.CurvatureAt(s));
  if (kappa_here > 1e-6) {
    target = std::min(target, std::sqrt(options_.a_lat_max_mps2 / kappa_here));
  }
  // Stop profile toward the end of the line.
  const double remaining = std::max(0.0, line_.length() - s);
  target =
      std::min(target, std::sqrt(2.0 * options_.end_decel_mps2 * remaining));
  return std::max(0.0, target);
}

// Time derivative of the profile followed at its own speed: by the chain
// rule dv/dt = (dv/ds)(ds/dt) = v dv/ds. On a braking ramp
// v(s) = sqrt(v_b^2 + 2 a (s_b - s)) one has dv/ds = -a / v, so v dv/ds is
// exactly -a: the feed-forward asks for the planned deceleration itself and
// the PID only corrects the residual. The slope is a forward difference
// over ds = max(0.5 m, v dt), i.e. at least one sample spacing and at least
// one tick of travel. Clamped to the vehicle limits; 0 at the line end.
double PurePursuitPid::ProfileAccel(double s, double dt_s) const {
  // Feed-forward from the speed profile: dv/dt = v dv/ds along it, which on a
  // braking ramp sqrt(v_b^2 + 2 a (s_b - s)) is exactly -a. The PID alone
  // would trail the ramp by kp^-1 m/s^2 of error. Read over at least one
  // sample so the difference is not noise between neighbours.
  const double v_here = TargetSpeed(s);
  const double ds = std::max(0.5, v_here * std::max(dt_s, 0.0));
  const double s_ahead = std::min(s + ds, line_.length());
  if (s_ahead <= s) {
    return 0.0;
  }
  const double slope = (TargetSpeed(s_ahead) - v_here) / (s_ahead - s);
  return std::clamp(v_here * slope, model_.limits.a_min_mps2,
                    model_.limits.a_max_mps2);
}

// Pure pursuit (header): L_d = clamp(k_v v + L_0, min, max), target = line
// point at s + L_d, alpha = angle from the heading to the chord, and
// delta = atan(2 L sin(alpha) / chord) with L = L_fit + K v^2. Then the
// actuator constraints: |delta| <= max_steer_angle and
// |delta - delta_prev| <= steer_rate_max dt. Two guards keep the geometry
// finite: L is floored at half the geometric wheelbase (a negative fitted
// K must not shrink it toward 0 at speed) and the chord at 0.5 m (the ego
// sitting on the target would otherwise divide by 0). sin(alpha) rather
// than alpha keeps the steer bounded and sign-correct for |alpha| > pi/2
// (a target behind the car asks for full lock toward it).
double PurePursuitPid::LateralStep(const nuway_common::SE2& pose,
                                   double speed_mps, double s, double dt_s,
                                   ControlOutput* out) {
  const double lookahead =
      std::clamp((options_.lookahead_gain_s * std::max(0.0, speed_mps)) +
                     options_.lookahead_base_m,
                 options_.lookahead_min_m, options_.lookahead_max_m);
  const double wheelbase =
      std::max(0.5 * model_.wheelbase_m, model_.EffectiveWheelbaseM(speed_mps));
  const double s_target = s + lookahead;
  nuway_common::CartesianPoint target = line_.PointAt(s_target);
  if (s_target > line_.length()) {
    // PointAt clamps to the last sample; as the ego closes in on it the
    // chord shrinks and the geometry degenerates into full lock while the
    // car creeps the last metre. Continue the line straight past its end.
    const double extra = s_target - line_.length();
    target.x += extra * std::cos(target.heading);
    target.y += extra * std::sin(target.heading);
  }
  const double dx = target.x - pose.x;
  const double dy = target.y - pose.y;
  const double chord = std::max(0.5, std::hypot(dx, dy));
  const double alpha = nuway_common::WrapAngle(std::atan2(dy, dx) - pose.yaw);
  double steer = std::atan(2.0 * wheelbase * std::sin(alpha) / chord);
  steer = std::clamp(steer, -model_.max_steer_angle_rad,
                     model_.max_steer_angle_rad);
  const double max_step = model_.limits.steer_rate_max_radps * dt_s;
  steer =
      std::clamp(steer, prev_steer_rad_ - max_step, prev_steer_rad_ + max_step);
  prev_steer_rad_ = steer;
  out->lookahead_m = lookahead;
  return steer;
}

// Speed loop: a = ff + kp e + ki I + kd D with e = target - v.
//   P   reacts to the current error; alone it leaves a steady offset (the
//       drag the map does not model, a grade) of a_residual / kp.
//   I   accumulates the error (I += e dt) and removes that offset over
//       time. Its bound integral_limit / ki keeps |ki I| <= integral_limit
//       and it is only committed when the output is not saturated in the
//       direction of the error (conditional integration): while the car is
//       already at full throttle, integrating the remaining error would
//       wind the term up and overshoot once the error closes.
//   D   damps: D = -dv/dt, the derivative of the *measured* speed, which
//       equals de/dt when the target is constant and skips the target's
//       jumps otherwise (a limit change, a bend entering the horizon would
//       kick the output by kd * step / dt for one tick).
//   ff  the profile's own v dv/ds (ProfileAccel), so on a planned braking
//       ramp the PID starts from the right deceleration.
// The sum is clamped to the vehicle limits and, when the model has one, to
// what the LongitudinalMap can deliver at v (full throttle / full brake),
// so the adapter's inverse never saturates silently. At a standstill with a
// zero target the output is forced to at most -end_decel so the brake holds
// (0 m/s^2 would be throttle against the idle drag, see below).
double PurePursuitPid::LongitudinalStep(double speed_mps, double target_mps,
                                        double feedforward_mps2, double dt_s) {
  const double error = target_mps - speed_mps;
  // Derivative on the measurement, not the error: a target step (a limit
  // change at a lane boundary, a bend entering the horizon) would otherwise
  // kick the output by kd * step / dt for one tick.
  double derivative = 0.0;
  if (prev_speed_mps_.has_value() && dt_s > 0.0) {
    derivative = -(speed_mps - *prev_speed_mps_) / dt_s;
  }
  prev_speed_mps_ = speed_mps;
  const double integral_bound =
      options_.ki > 0.0 ? options_.integral_limit_mps2 / options_.ki : 0.0;
  const double proposed =
      std::clamp(integral_ + (error * dt_s), -integral_bound, integral_bound);
  const double accel = feedforward_mps2 + (options_.kp * error) +
                       (options_.ki * proposed) + (options_.kd * derivative);
  double lo = model_.limits.a_min_mps2;
  double hi = model_.limits.a_max_mps2;
  if (model_.longitudinal_map.has_value()) {
    const LongitudinalMap& map = *model_.longitudinal_map;
    lo = std::max(lo, map.Decel(speed_mps, 1.0));
    hi = std::min(hi, map.Accel(speed_mps, 1.0));
  }
  const double clamped = std::clamp(accel, lo, hi);
  // Conditional integration: hold the integral while the output saturates in
  // the direction of the error (integrating against the saturation, which
  // unwinds the term, is still allowed).
  const bool saturated =
      (clamped < accel && error > 0.0) || (clamped > accel && error < 0.0);
  if (!saturated) {
    integral_ = proposed;
  }
  // Standstill at the target: hold the brake. The adapter splits at the
  // coast deceleration, so 0 m/s^2 at rest would be throttle against the
  // idle drag; end_decel_mps2 is the stop profile's own deceleration.
  if (target_mps <= 0.05 && speed_mps < 0.1) {
    integral_ = 0.0;
    return std::min(clamped, -options_.end_decel_mps2);
  }
  return clamped;
}

// One tick: project the rear axle onto the line, decide whether the car is
// in a state the controller can handle, then run the two loops. Every early
// return hands back the default ControlOutput, whose emergency_stop = true
// is the no-input output (steer 0, brake 1 at the adapter), and drops the
// loop memories because the actuators are no longer where the loops left
// them. The past-the-end check keeps the brake held once the stop profile
// has brought the car to the last sample (the line runs 50 m past the goal,
// so the route is complete long before that).
ControlOutput PurePursuitPid::Step(const nuway_common::SE2& pose,
                                   double speed_mps, double dt_s) {
  ControlOutput out;
  if (!has_line_ || line_.size() < 2) {
    ResetTransients();
    return out;
  }
  // Project near the previous s first; the global search is the fallback
  // for the first tick on a line and after a teleport. Both fail (nullopt)
  // when the ego is farther than max_lateral_error_m from the line.
  std::optional<nuway_common::FrenetPoint> frenet;
  if (last_s_m_.has_value()) {
    frenet = line_.ToFrenetNear(pose.x, pose.y, options_.max_lateral_error_m,
                                *last_s_m_, options_.projection_back_m,
                                options_.projection_ahead_m);
  }
  if (!frenet.has_value()) {
    frenet = line_.ToFrenet(pose.x, pose.y, options_.max_lateral_error_m);
  }
  if (!frenet.has_value()) {
    ResetTransients();
    return out;
  }
  last_s_m_ = frenet->s;
  out.s_m = frenet->s;
  out.lateral_error_m = frenet->d;
  out.heading_error_rad =
      nuway_common::WrapAngle(line_.HeadingAt(frenet->s) - pose.yaw);
  if (frenet->s >= line_.length() - 0.1) {
    ResetTransients();
    return out;  // past the end: hold the brake
  }
  out.emergency_stop = false;
  out.target_speed_mps = TargetSpeed(frenet->s);
  out.speed_error_mps = out.target_speed_mps - speed_mps;
  out.steering_angle_rad = LateralStep(pose, speed_mps, frenet->s, dt_s, &out);
  out.accel_mps2 = LongitudinalStep(speed_mps, out.target_speed_mps,
                                    ProfileAccel(frenet->s, dt_s), dt_s);
  return out;
}

}  // namespace nuway_control
