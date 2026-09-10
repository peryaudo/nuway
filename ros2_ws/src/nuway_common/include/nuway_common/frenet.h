// Reference line and Cartesian <-> Frenet conversion. A ReferenceLine is a
// polyline sampled along arc length s (meters, map frame) with per-sample
// heading (rad) and curvature (1/m); Frenet coordinates are (s, d) with d
// positive to the left of the line (ROS convention). Mirrored by
// nuway_ml/common/frenet.py (parity-tested). Introduced in M0; the state
// conversions (velocity and acceleration projection) arrive with M1.
//
// Why Frenet. A road is a curve; describing motion relative to it turns
// "stay in lane, keep the speed" into two one-dimensional problems: s(t)
// along the line and d(s) across it. The lattice planner (M1 §3.3) samples
// polynomials in exactly those coordinates and needs to move each candidate
// back into the map frame with a consistent heading, speed, acceleration
// and curvature for the collision checker and the controller. The formulas
// (ToCartesianState / ToFrenetState) are the standard ones of Werling et al.,
// "Optimal trajectory generation for dynamic street scenarios in a Frenet
// frame" (ICRA 2010), in the form Apollo's CartesianFrenetConverter uses:
// with the projection distance d, the heading difference dtheta = theta -
// theta_r, and one_minus_kd = 1 - kappa_r d (the metric stretch: a point
// left of a left bend travels a shorter arc than the line),
//   s_dot = v cos(dtheta) / one_minus_kd
//   d'    = one_minus_kd tan(dtheta)                (d' = dd/ds)
// and the second derivatives follow by differentiating once more, which
// brings in the curvature rate kappa_r' along the line.
#ifndef NUWAY_COMMON_FRENET_H_
#define NUWAY_COMMON_FRENET_H_

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "nuway_common/geometry.h"

namespace nuway_common {

// A point on the line in Frenet coordinates.
struct FrenetPoint {
  double s = 0.0;  // arc length along the line, m
  double d = 0.0;  // signed lateral offset, +left, m
};

// A Cartesian point with the line heading at its projection.
struct CartesianPoint {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;  // rad, heading of the line at s
};

// The full Frenet state of a moving point (M1): s and its time derivatives,
// d and its derivatives with respect to s (d' = dd/ds, d'' = d^2d/ds^2).
struct FrenetState {
  double s = 0.0;
  double s_dot = 0.0;   // m/s
  double s_ddot = 0.0;  // m/s^2
  double d = 0.0;
  double d_prime = 0.0;   // dd/ds, dimensionless
  double d_dprime = 0.0;  // d^2d/ds^2, 1/m
};

// The full Cartesian state of a moving point (M1): pose, speed along its
// own heading, tangential acceleration and path curvature.
struct CartesianState {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;    // rad
  double v = 0.0;      // m/s along yaw
  double a = 0.0;      // m/s^2 along the path
  double kappa = 0.0;  // 1/m, positive left
};

// Polyline reference line. Samples must be strictly increasing in s.
class ReferenceLine {
 public:
  ReferenceLine() = default;

  // Builds from sample points; s is the cumulative chord length, heading the
  // central-difference direction (one-sided at the ends) and curvature the
  // signed Menger curvature of consecutive triples (zero at the ends).
  static ReferenceLine FromPoints(const Vector2dList& points) {
    ReferenceLine line;
    line.points_ = points;
    const int n = static_cast<int>(points.size());
    line.s_.assign(points.size(), 0.0);
    line.heading_.assign(points.size(), 0.0);
    line.curvature_.assign(points.size(), 0.0);
    for (int i = 1; i < n; ++i) {
      line.s_[i] = line.s_[i - 1] + Distance(points[i - 1], points[i]);
    }
    for (int i = 0; i < n; ++i) {
      const int prev = std::max(0, i - 1);
      const int next = std::min(n - 1, i + 1);
      if (prev == next) {
        continue;
      }
      const Eigen::Vector2d delta = points[next] - points[prev];
      line.heading_[i] = std::atan2(delta.y(), delta.x());
    }
    for (int i = 1; i + 1 < n; ++i) {
      line.curvature_[i] =
          MengerCurvature(points[i - 1], points[i], points[i + 1]);
    }
    return line;
  }

  // Builds from explicit samples (e.g. a ReferenceLine message).
  static ReferenceLine FromSamples(const Vector2dList& points,
                                   const std::vector<double>& s,
                                   const std::vector<double>& heading,
                                   const std::vector<double>& curvature) {
    ReferenceLine line;
    line.points_ = points;
    line.s_ = s;
    line.heading_ = heading;
    line.curvature_ = curvature;
    return line;
  }

  // Signed Menger curvature of three points: positive for a left turn.
  static double MengerCurvature(const Eigen::Vector2d& first,
                                const Eigen::Vector2d& mid,
                                const Eigen::Vector2d& last) {
    const Eigen::Vector2d first_mid = mid - first;
    const Eigen::Vector2d mid_last = last - mid;
    const Eigen::Vector2d first_last = last - first;
    const double cross =
        (first_mid.x() * mid_last.y()) - (first_mid.y() * mid_last.x());
    const double denom = first_mid.norm() * mid_last.norm() * first_last.norm();
    if (denom < 1e-12) {
      return 0.0;
    }
    return 2.0 * cross / denom;
  }

  int size() const { return static_cast<int>(points_.size()); }
  bool empty() const { return points_.empty(); }
  double length() const { return s_.empty() ? 0.0 : s_.back(); }
  const Vector2dList& points() const { return points_; }
  const std::vector<double>& s() const { return s_; }
  const std::vector<double>& heading() const { return heading_; }
  const std::vector<double>& curvature() const { return curvature_; }

  // Index of the segment [i, i+1] containing s (clamped to the line).
  int SegmentIndex(double s) const {
    if (size() < 2) {
      return 0;
    }
    const auto upper = std::upper_bound(s_.begin(), s_.end(), s);
    int idx = static_cast<int>(upper - s_.begin()) - 1;
    idx = std::max(0, std::min(idx, size() - 2));
    return idx;
  }

  // Line point at arc length s (linear interpolation, clamped to the line).
  CartesianPoint PointAt(double s) const {
    return ToCartesian(FrenetPoint{s, 0.0});
  }

  // Heading of the line at arc length s, interpolated on the circle.
  double HeadingAt(double s) const {
    if (size() < 2) {
      return heading_.empty() ? 0.0 : heading_.front();
    }
    const int idx = SegmentIndex(s);
    const double alpha = SegmentFraction(idx, s);
    return WrapAngle(heading_[idx] +
                     (alpha * WrapAngle(heading_[idx + 1] - heading_[idx])));
  }

  // Curvature of the line at arc length s (linear interpolation).
  double CurvatureAt(double s) const {
    if (size() < 2) {
      return curvature_.empty() ? 0.0 : curvature_.front();
    }
    const int idx = SegmentIndex(s);
    const double alpha = SegmentFraction(idx, s);
    return curvature_[idx] + (alpha * (curvature_[idx + 1] - curvature_[idx]));
  }

  // Curvature rate dkappa/ds at arc length s: a central difference over
  // kCurvatureRateStepM, clamped to the line. The Menger curvature of a
  // polyline is piecewise linear, so this is the slope of the interpolant.
  double CurvatureRateAt(double s) const {
    if (size() < 2) {
      return 0.0;
    }
    const double lo = std::max(0.0, s - kCurvatureRateStepM);
    const double hi = std::min(length(), s + kCurvatureRateStepM);
    if (hi - lo <= 0.0) {
      return 0.0;
    }
    return (CurvatureAt(hi) - CurvatureAt(lo)) / (hi - lo);
  }

  // Frenet state -> Cartesian state (Werling 2010 / Apollo
  // frenet_to_cartesian). The point is p(s) + d n(s); its heading is the
  // line heading plus dtheta = atan2(d', 1 - kappa_r d); speed, curvature
  // and acceleration follow from differentiating the position twice:
  //   v     = s_dot one_minus_kd / cos(dtheta)
  //   kappa = ((d'' + (kappa_r' d + kappa_r d') tan(dtheta)) cos^2(dtheta)
  //            / one_minus_kd + kappa_r) cos(dtheta) / one_minus_kd
  //   a     = s_ddot one_minus_kd / cos(dtheta)
  //           + s_dot^2 / cos(dtheta) (d' (kappa one_minus_kd / cos(dtheta)
  //             - kappa_r) - (kappa_r' d + kappa_r d'))
  // Valid while |d| kappa_r < 1 (the point is inside the line's centre of
  // curvature otherwise and the map is singular).
  CartesianState ToCartesianState(const FrenetState& f) const {
    const CartesianPoint base = ToCartesian(FrenetPoint{f.s, f.d});
    const double kappa_r = CurvatureAt(f.s);
    const double dkappa_r = CurvatureRateAt(f.s);
    const double one_minus_kd = 1.0 - (kappa_r * f.d);
    const double dtheta = std::atan2(f.d_prime, one_minus_kd);
    const double cos_dtheta = std::cos(dtheta);
    const double tan_dtheta = std::tan(dtheta);
    const double kd_term = (dkappa_r * f.d) + (kappa_r * f.d_prime);
    CartesianState out;
    out.x = base.x;
    out.y = base.y;
    out.yaw = WrapAngle(base.heading + dtheta);
    out.v = f.s_dot * one_minus_kd / cos_dtheta;
    out.kappa =
        ((((f.d_dprime + (kd_term * tan_dtheta)) * cos_dtheta * cos_dtheta) /
          one_minus_kd) +
         kappa_r) *
        cos_dtheta / one_minus_kd;
    const double delta_theta_prime =
        ((out.kappa * one_minus_kd) / cos_dtheta) - kappa_r;
    out.a = ((f.s_ddot * one_minus_kd) / cos_dtheta) +
            ((f.s_dot * f.s_dot / cos_dtheta) *
             ((f.d_prime * delta_theta_prime) - kd_term));
    return out;
  }

  // Cartesian state -> Frenet state at the global projection of (x, y);
  // nullopt when the projection fails (see ToFrenet).
  std::optional<FrenetState> ToFrenetState(const CartesianState& c,
                                           double max_dist = 1e9) const {
    const std::optional<FrenetPoint> point = ToFrenet(c.x, c.y, max_dist);
    if (!point.has_value()) {
      return std::nullopt;
    }
    return FrenetStateAt(c, *point);
  }

  // Cartesian state -> Frenet state with the windowed projection of
  // ToFrenetNear (a follower's previous s as the hint).
  std::optional<FrenetState> ToFrenetStateNear(const CartesianState& c,
                                               double max_dist, double s_hint,
                                               double back_m,
                                               double ahead_m) const {
    const std::optional<FrenetPoint> point =
        ToFrenetNear(c.x, c.y, max_dist, s_hint, back_m, ahead_m);
    if (!point.has_value()) {
      return std::nullopt;
    }
    return FrenetStateAt(c, *point);
  }

  // The derivative part of the Cartesian -> Frenet conversion at a known
  // projection (Werling 2010 / Apollo cartesian_to_frenet):
  //   s_dot  = v cos(dtheta) / one_minus_kd
  //   d'     = one_minus_kd tan(dtheta)
  //   d''    = -(kappa_r' d + kappa_r d') tan(dtheta)
  //            + one_minus_kd / cos^2(dtheta)
  //              (kappa one_minus_kd / cos(dtheta) - kappa_r)
  //   s_ddot = (a cos(dtheta) - s_dot^2 (d' delta_theta' - (kappa_r' d
  //             + kappa_r d'))) / one_minus_kd
  FrenetState FrenetStateAt(const CartesianState& c,
                            const FrenetPoint& point) const {
    const double theta_r = HeadingAt(point.s);
    const double kappa_r = CurvatureAt(point.s);
    const double dkappa_r = CurvatureRateAt(point.s);
    const double dtheta = WrapAngle(c.yaw - theta_r);
    const double cos_dtheta = std::cos(dtheta);
    const double tan_dtheta = std::tan(dtheta);
    const double one_minus_kd = 1.0 - (kappa_r * point.d);
    FrenetState out;
    out.s = point.s;
    out.d = point.d;
    out.s_dot = c.v * cos_dtheta / one_minus_kd;
    out.d_prime = one_minus_kd * tan_dtheta;
    const double kd_term = (dkappa_r * point.d) + (kappa_r * out.d_prime);
    const double delta_theta_prime =
        ((c.kappa * one_minus_kd) / cos_dtheta) - kappa_r;
    out.d_dprime =
        (-kd_term * tan_dtheta) +
        ((one_minus_kd / (cos_dtheta * cos_dtheta)) * delta_theta_prime);
    out.s_ddot =
        ((c.a * cos_dtheta) - (out.s_dot * out.s_dot *
                               ((out.d_prime * delta_theta_prime) - kd_term))) /
        one_minus_kd;
    return out;
  }

  // Frenet -> Cartesian: p(s) + d * n(s) with n the left normal of the
  // interpolated heading. s is clamped to [0, length].
  CartesianPoint ToCartesian(const FrenetPoint& frenet) const {
    if (empty()) {
      return CartesianPoint{};
    }
    if (size() == 1) {
      return CartesianPoint{points_[0].x(), points_[0].y() + frenet.d, 0.0};
    }
    const Eigen::Vector2d base = Interpolate(frenet.s);
    const double heading = HeadingAt(frenet.s);
    const Eigen::Vector2d normal(-std::sin(heading), std::cos(heading));
    const Eigen::Vector2d out = base + (frenet.d * normal);
    return CartesianPoint{out.x(), out.y(), heading};
  }

  // Cartesian -> Frenet: the s at which (q - p(s)) is normal to the heading,
  // found by a nearest-segment projection refined with Newton steps; d is
  // the signed offset along that normal. Returns nullopt if the line has
  // fewer than two points or the point is farther than max_dist from every
  // segment. Exact inverse of ToCartesian while |d| * curvature < 1.
  std::optional<FrenetPoint> ToFrenet(double x, double y,
                                      double max_dist = 1e9) const {
    if (size() < 2) {
      return std::nullopt;
    }
    return Project(Eigen::Vector2d(x, y), max_dist, 0, size() - 1);
  }

  // ToFrenet restricted to the part of the line with s within
  // [s_hint - back_m, s_hint + ahead_m]. Where a route crosses or loops back
  // near itself the global nearest segment can lie on another leg; a
  // follower that knows roughly where it is (its last s) searches only around
  // there. Returns nullopt when nothing in the window is within max_dist;
  // callers then fall back to ToFrenet() (after a reset or teleport).
  std::optional<FrenetPoint> ToFrenetNear(double x, double y, double max_dist,
                                          double s_hint, double back_m,
                                          double ahead_m) const {
    if (size() < 2) {
      return std::nullopt;
    }
    const int first = SegmentIndex(s_hint - back_m);
    const int last = SegmentIndex(s_hint + ahead_m) + 1;
    return Project(Eigen::Vector2d(x, y), max_dist, first, last);
  }

 private:
  static constexpr int kNewtonIterations = 12;
  static constexpr double kNewtonTolerance = 1e-10;
  // Half-width of the central difference behind CurvatureRateAt: the
  // reference line is sampled every 0.5 m, so a wider step only blurs.
  static constexpr double kCurvatureRateStepM = 0.5;

  // The projection itself over the segments [first, last): the nearest
  // segment gives the start, Newton steps on the tangent condition refine it.
  std::optional<FrenetPoint> Project(const Eigen::Vector2d& query,
                                     double max_dist, int first,
                                     int last) const {
    const std::optional<double> coarse =
        NearestSegmentS(query, max_dist, first, last);
    if (!coarse.has_value()) {
      return std::nullopt;
    }
    double s = *coarse;
    for (int iter = 0; iter < kNewtonIterations; ++iter) {
      const Eigen::Vector2d diff = query - Interpolate(s);
      const double heading = HeadingAt(s);
      const Eigen::Vector2d tangent(std::cos(heading), std::sin(heading));
      const Eigen::Vector2d normal(-tangent.y(), tangent.x());
      const double along = diff.dot(tangent);
      const double lateral = diff.dot(normal);
      const double slope = 1.0 - (lateral * CurvatureAt(s));
      const double step = std::abs(slope) > 0.1 ? along / slope : along;
      const double next = std::max(0.0, std::min(length(), s + step));
      const bool converged = std::abs(next - s) < kNewtonTolerance;
      s = next;
      if (converged) {
        break;
      }
    }
    const Eigen::Vector2d diff = query - Interpolate(s);
    const double heading = HeadingAt(s);
    const Eigen::Vector2d normal(-std::sin(heading), std::cos(heading));
    return FrenetPoint{s, diff.dot(normal)};
  }

  double SegmentFraction(int idx, double s) const {
    const double seg_len = s_[idx + 1] - s_[idx];
    if (seg_len <= 0.0) {
      return 0.0;
    }
    return std::max(0.0, std::min(1.0, (s - s_[idx]) / seg_len));
  }

  Eigen::Vector2d Interpolate(double s) const {
    const int idx = SegmentIndex(s);
    const double alpha = SegmentFraction(idx, s);
    return points_[idx] + (alpha * (points_[idx + 1] - points_[idx]));
  }

  // Arc length of the closest point on the segments [first, last) of the
  // polyline, or nullopt beyond max_dist.
  std::optional<double> NearestSegmentS(const Eigen::Vector2d& query,
                                        double max_dist, int first,
                                        int last) const {
    double best_dist2 = max_dist * max_dist;
    std::optional<double> best;
    for (int i = std::max(0, first); i < std::min(last, size() - 1); ++i) {
      const Eigen::Vector2d& p0 = points_[i];
      const Eigen::Vector2d seg = points_[i + 1] - p0;
      const double seg_len2 = seg.squaredNorm();
      double alpha = 0.0;
      if (seg_len2 > 0.0) {
        alpha = std::max(0.0, std::min(1.0, (query - p0).dot(seg) / seg_len2));
      }
      const Eigen::Vector2d proj = p0 + (alpha * seg);
      const double dist2 = (query - proj).squaredNorm();
      if (dist2 < best_dist2) {
        best_dist2 = dist2;
        best = s_[i] + (alpha * std::sqrt(seg_len2));
      }
    }
    return best;
  }

  Vector2dList points_;
  std::vector<double> s_;
  std::vector<double> heading_;
  std::vector<double> curvature_;
};

}  // namespace nuway_common

#endif  // NUWAY_COMMON_FRENET_H_
