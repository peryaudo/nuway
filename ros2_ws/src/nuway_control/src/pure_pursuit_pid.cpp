#include "nuway_control/pure_pursuit_pid.hpp"

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
}

void PurePursuitPid::Reset() {
  line_ = nuway_common::ReferenceLine{};
  has_line_ = false;
  speed_limit_mps_.clear();
  integral_ = 0.0;
  prev_speed_error_.reset();
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
  // Curvature bound over the horizon, on the line samples plus s itself.
  const std::vector<double>& line_s = line_.s();
  const double s_end =
      std::min(line_.length(), s + options_.curvature_horizon_m);
  const auto first = std::lower_bound(line_s.begin(), line_s.end(), s);
  for (auto it = first; it != line_s.end() && *it <= s_end; ++it) {
    const std::size_t i = static_cast<std::size_t>(it - line_s.begin());
    const double kappa = std::abs(line_.curvature()[i]);
    if (kappa > 1e-6) {
      target = std::min(target, std::sqrt(options_.a_lat_max_mps2 / kappa));
    }
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

double PurePursuitPid::LateralStep(const nuway_common::SE2& pose,
                                   double speed_mps, double s, double dt_s,
                                   ControlOutput* out) {
  const double lookahead =
      std::clamp((options_.lookahead_gain_s * std::max(0.0, speed_mps)) +
                     options_.lookahead_base_m,
                 options_.lookahead_min_m, options_.lookahead_max_m);
  const nuway_common::CartesianPoint target = line_.PointAt(s + lookahead);
  const double dx = target.x - pose.x;
  const double dy = target.y - pose.y;
  const double chord = std::max(0.5, std::hypot(dx, dy));
  const double alpha = nuway_common::WrapAngle(std::atan2(dy, dx) - pose.yaw);
  const double wheelbase =
      std::max(0.5 * model_.wheelbase_m, model_.EffectiveWheelbaseM(speed_mps));
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
                                        double dt_s) {
  const double error = target_mps - speed_mps;
  double derivative = 0.0;
  if (prev_speed_error_.has_value() && dt_s > 0.0) {
    derivative = (error - *prev_speed_error_) / dt_s;
  }
  prev_speed_error_ = error;
  const double integral_bound =
      options_.ki > 0.0 ? options_.integral_limit_mps2 / options_.ki : 0.0;
  const double proposed =
      std::clamp(integral_ + (error * dt_s), -integral_bound, integral_bound);
  const double accel = (options_.kp * error) + (options_.ki * proposed) +
                       (options_.kd * derivative);
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
  // Standstill at the target: no creeping from integral history.
  if (target_mps <= 0.05 && speed_mps < 0.1) {
    integral_ = 0.0;
    return std::min(clamped, 0.0);
  }
  return clamped;
}

ControlOutput PurePursuitPid::Step(const nuway_common::SE2& pose,
                                   double speed_mps, double dt_s) {
  ControlOutput out;
  if (!has_line_ || line_.size() < 2) {
    return out;
  }
  const std::optional<nuway_common::FrenetPoint> frenet =
      line_.ToFrenet(pose.x, pose.y, options_.max_lateral_error_m);
  if (!frenet.has_value()) {
    return out;
  }
  out.s_m = frenet->s;
  out.lateral_error_m = frenet->d;
  out.heading_error_rad =
      nuway_common::WrapAngle(line_.HeadingAt(frenet->s) - pose.yaw);
  if (frenet->s >= line_.length() - 0.1) {
    return out;  // past the end: hold the brake
  }
  out.emergency_stop = false;
  out.target_speed_mps = TargetSpeed(frenet->s);
  out.speed_error_mps = out.target_speed_mps - speed_mps;
  out.steering_angle_rad = LateralStep(pose, speed_mps, frenet->s, dt_s, &out);
  out.accel_mps2 = LongitudinalStep(speed_mps, out.target_speed_mps, dt_s);
  return out;
}

}  // namespace nuway_control
