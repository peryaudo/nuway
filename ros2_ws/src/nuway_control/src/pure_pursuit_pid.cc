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

void PurePursuitPid::Reset() {
  line_ = nuway_common::ReferenceLine{};
  has_line_ = false;
  speed_limit_mps_.clear();
  ResetTransients();
  last_s_m_.reset();
}

void PurePursuitPid::ResetTransients() {
  integral_ = 0.0;
  prev_speed_mps_.reset();
  prev_steer_rad_ = 0.0;
}

double PurePursuitPid::SpeedLimitAt(double s) const {
  if (speed_limit_mps_.empty()) {
    return 0.0;
  }
  const auto idx = static_cast<std::size_t>(line_.SegmentIndex(s));
  const std::size_t next = std::min(idx + 1, speed_limit_mps_.size() - 1);
  return std::min(speed_limit_mps_[idx], speed_limit_mps_[next]);
}

double PurePursuitPid::TargetSpeed(double s) const {
  double target = SpeedLimitAt(s);
  // Bounds over the horizon: a bend or a lower limit ahead caps the speed now
  // to what a plan_decel_mps2 braking reaches it with, so the profile is
  // continuous and its slope is a usable feed-forward. Continuity needs the
  // horizon to reach v_limit^2 / (2 a): a bound entering a shorter horizon
  // caps the speed in one step (24.6 -> 15.8 m/s at 60 m), and beyond that
  // distance sqrt(2 a ds) alone already exceeds the limit.
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
    if (kappa > 1e-6) {
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
  // the direction of the error.
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

ControlOutput PurePursuitPid::Step(const nuway_common::SE2& pose,
                                   double speed_mps, double dt_s) {
  ControlOutput out;
  if (!has_line_ || line_.size() < 2) {
    ResetTransients();
    return out;
  }
  // Project near the previous s first; the global search is the fallback
  // for the first tick on a line and after a teleport.
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
