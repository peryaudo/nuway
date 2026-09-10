// The route's reference line as the planning libraries see it (M1): the
// geometric nuway_common::ReferenceLine plus the per-sample attributes of
// nuway_msgs/ReferenceLine (lane id, speed limit, drivable bounds) and the
// goal arc length, with the speed-profile query the FSM's target speed and
// the lattice's speed limit share. Built once per episode from the latched
// message (msg_conv.h); no rclcpp.
#ifndef NUWAY_PLANNING_ROUTE_LINE_H_
#define NUWAY_PLANNING_ROUTE_LINE_H_

#include <cstdint>
#include <optional>
#include <vector>

#include <nuway_common/frenet.h>
#include <nuway_common/geometry.h>

namespace nuway_planning {

// Speed-profile knobs shared by the FSM target speed and the lattice speed
// bound; the defaults are the M0 task 13 tuning (M0 §2.7).
struct SpeedProfileOptions {
  double a_lat_max_mps2 = 2.0;   // curvature speed = sqrt(a_lat_max / |kappa|)
  double plan_decel_mps2 = 2.0;  // braking assumed on the approach to a bound
  double curvature_horizon_m = 60.0;  // floor of the look-ahead
};

// Drivable extent at an arc length: distance to the edge on each side,
// positive (docs/02 §4 ReferenceLine.left_bound / right_bound).
struct LateralBounds {
  double left_m = 1.75;
  double right_m = 1.75;
};

class RouteLine {
 public:
  RouteLine() = default;
  // `line` with one entry per sample in every attribute vector (shorter
  // vectors repeat their last value, empty ones a default: lane 0, 0 m/s,
  // 1.75 m bounds). goal_s is the arc length of the route goal (the line
  // continues extension_m past it, M0 §2.5); nullopt puts the goal at the
  // line end.
  RouteLine(nuway_common::ReferenceLine line,
            std::vector<std::uint32_t> lane_id,
            std::vector<double> speed_limit_mps,
            std::vector<double> left_bound_m, std::vector<double> right_bound_m,
            std::optional<double> goal_s);

  const nuway_common::ReferenceLine& line() const { return line_; }
  bool empty() const { return line_.size() < 2; }
  double length() const { return line_.length(); }
  double goal_s() const { return goal_s_; }
  const std::vector<std::uint32_t>& lane_ids() const { return lane_id_; }

  // Route lane at arc length s (the sample at or before s).
  std::uint32_t LaneIdAt(double s) const;
  // Posted limit at s: the lower of the two bracketing samples, so a drop at
  // the next sample already applies (conservative, at most 0.5 m early).
  double SpeedLimitAt(double s) const;
  // Drivable extent at s (the sample at or before s).
  LateralBounds BoundsAt(double s) const;
  // True when `lane_id` is a route lane at or after arc length s.
  bool IsRouteLaneAhead(std::uint32_t lane_id, double s) const;

  // The speed profile v*(s) of M0 §2.7 (PurePursuitPid::TargetSpeed) without
  // the end stop: min of the posted limit at s, the curvature speed at s,
  // and for every sample s' within max(curvature_horizon,
  // v_limit^2 / (2 plan_decel)) the speed reachable now that still meets
  // min(limit(s'), curvature speed(s')) at s' when braking at plan_decel
  // (v^2 = v_0^2 + 2 a d). The horizon rule keeps the profile continuous:
  // beyond that distance no bound can cap the speed below the limit.
  double SpeedBoundAt(double s, const SpeedProfileOptions& options) const;

  // The curvature speed cap over the stretch [s, s + reach_m]: the lowest
  // sqrt(a_lat_max / |kappa|) of the samples there (and of s itself);
  // +infinity on a straight. What a speed profile that runs through the
  // stretch at a constant end speed may aim at without exceeding a_lat_max
  // (the lattice's keep targets, M1 §3.3 feasibility filter).
  double CurvatureSpeedCap(double s, double reach_m, double a_lat_max) const;

  // Frenet projection with the windowed search of ReferenceLine::
  // ToFrenetNear when a hint is given, the global search otherwise, and the
  // global search as the fallback when the window finds nothing.
  std::optional<nuway_common::FrenetPoint> Project(double x, double y,
                                                   double max_dist,
                                                   std::optional<double> s_hint,
                                                   double back_m,
                                                   double ahead_m) const;

 private:
  std::size_t SampleIndexAt(double s) const;

  nuway_common::ReferenceLine line_;
  std::vector<std::uint32_t> lane_id_;
  std::vector<double> speed_limit_mps_;
  std::vector<double> left_bound_m_;
  std::vector<double> right_bound_m_;
  double goal_s_ = 0.0;
};

}  // namespace nuway_planning

#endif  // NUWAY_PLANNING_ROUTE_LINE_H_
