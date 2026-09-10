#include "nuway_planning/route_line.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace nuway_planning {

namespace {

// Brings an attribute vector to one entry per line sample.
template <typename T>
std::vector<T> Fit(std::vector<T> values, std::size_t n, T fallback) {
  const T fill = values.empty() ? fallback : values.back();
  values.resize(n, fill);
  return values;
}

}  // namespace

RouteLine::RouteLine(nuway_common::ReferenceLine line,
                     std::vector<std::uint32_t> lane_id,
                     std::vector<double> speed_limit_mps,
                     std::vector<double> left_bound_m,
                     std::vector<double> right_bound_m,
                     std::optional<double> goal_s)
    : line_(std::move(line)) {
  const auto n = static_cast<std::size_t>(line_.size());
  lane_id_ = Fit(std::move(lane_id), n, std::uint32_t{0});
  speed_limit_mps_ = Fit(std::move(speed_limit_mps), n, 0.0);
  left_bound_m_ = Fit(std::move(left_bound_m), n, 1.75);
  right_bound_m_ = Fit(std::move(right_bound_m), n, 1.75);
  goal_s_ = goal_s.value_or(line_.length());
}

std::size_t RouteLine::SampleIndexAt(double s) const {
  if (line_.size() < 2) {
    return 0;
  }
  return static_cast<std::size_t>(line_.SegmentIndex(s));
}

std::uint32_t RouteLine::LaneIdAt(double s) const {
  if (lane_id_.empty()) {
    return 0;
  }
  return lane_id_[SampleIndexAt(s)];
}

double RouteLine::SpeedLimitAt(double s) const {
  if (speed_limit_mps_.empty()) {
    return 0.0;
  }
  const std::size_t idx = SampleIndexAt(s);
  const std::size_t next = std::min(idx + 1, speed_limit_mps_.size() - 1);
  return std::min(speed_limit_mps_[idx], speed_limit_mps_[next]);
}

LateralBounds RouteLine::BoundsAt(double s) const {
  if (left_bound_m_.empty()) {
    return LateralBounds{};
  }
  const std::size_t idx = SampleIndexAt(s);
  return LateralBounds{left_bound_m_[idx], right_bound_m_[idx]};
}

bool RouteLine::IsRouteLaneAhead(std::uint32_t lane_id, double s) const {
  if (lane_id == 0 || lane_id_.empty()) {
    return false;
  }
  for (std::size_t i = SampleIndexAt(s); i < lane_id_.size(); ++i) {
    if (lane_id_[i] == lane_id) {
      return true;
    }
  }
  return false;
}

double RouteLine::SpeedBoundAt(double s,
                               const SpeedProfileOptions& options) const {
  if (line_.size() < 2) {
    return 0.0;
  }
  double target = SpeedLimitAt(s);
  const double v_limit = target;
  const double braking_distance =
      options.plan_decel_mps2 > 0.0
          ? (v_limit * v_limit) / (2.0 * options.plan_decel_mps2)
          : options.curvature_horizon_m;
  const double horizon =
      std::max(options.curvature_horizon_m, braking_distance);
  const std::vector<double>& line_s = line_.s();
  const double s_end = std::min(line_.length(), s + horizon);
  const auto first = std::lower_bound(line_s.begin(), line_s.end(), s);
  for (auto it = first; it != line_s.end() && *it <= s_end; ++it) {
    const auto i = static_cast<std::size_t>(it - line_s.begin());
    const double kappa = std::abs(line_.curvature()[i]);
    double v_there = SpeedLimitAt(*it);
    if (kappa > 1e-6) {  // straight: no lateral bound (1e-6 = R of 1000 km)
      v_there = std::min(v_there, std::sqrt(options.a_lat_max_mps2 / kappa));
    }
    const double v_here = std::sqrt(
        (v_there * v_there) + (2.0 * options.plan_decel_mps2 * (*it - s)));
    target = std::min(target, v_here);
  }
  const double kappa_here = std::abs(line_.CurvatureAt(s));
  if (kappa_here > 1e-6) {
    target = std::min(target, std::sqrt(options.a_lat_max_mps2 / kappa_here));
  }
  return std::max(0.0, target);
}

double RouteLine::CurvatureSpeedCap(double s, double reach_m,
                                    double a_lat_max) const {
  double cap = std::numeric_limits<double>::infinity();
  if (line_.size() < 2) {
    return cap;
  }
  const auto bound = [&](double kappa) {
    if (std::abs(kappa) > 1e-6) {  // 1e-6: R of 1000 km, a straight
      cap = std::min(cap, std::sqrt(a_lat_max / std::abs(kappa)));
    }
  };
  bound(line_.CurvatureAt(s));
  const std::vector<double>& line_s = line_.s();
  const double s_end = std::min(line_.length(), s + std::max(0.0, reach_m));
  const auto first = std::lower_bound(line_s.begin(), line_s.end(), s);
  for (auto it = first; it != line_s.end() && *it <= s_end; ++it) {
    bound(line_.curvature()[static_cast<std::size_t>(it - line_s.begin())]);
  }
  return cap;
}

std::optional<nuway_common::FrenetPoint> RouteLine::Project(
    double x, double y, double max_dist, std::optional<double> s_hint,
    double back_m, double ahead_m) const {
  if (s_hint.has_value()) {
    const std::optional<nuway_common::FrenetPoint> near =
        line_.ToFrenetNear(x, y, max_dist, *s_hint, back_m, ahead_m);
    if (near.has_value()) {
      return near;
    }
  }
  return line_.ToFrenet(x, y, max_dist);
}

}  // namespace nuway_planning
