// Reference line and Cartesian <-> Frenet conversion. A ReferenceLine is a
// polyline sampled along arc length s (meters, map frame) with per-sample
// heading (rad) and curvature (1/m); Frenet coordinates are (s, d) with d
// positive to the left of the line (ROS convention). Mirrored by
// nuway_ml/common/frenet.py (parity-tested). Introduced in M0.
#ifndef NUWAY_COMMON_FRENET_HPP_
#define NUWAY_COMMON_FRENET_HPP_

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "nuway_common/geometry.hpp"

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
    const Eigen::Vector2d query(x, y);
    const std::optional<double> coarse = NearestSegmentS(query, max_dist);
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

 private:
  static constexpr int kNewtonIterations = 12;
  static constexpr double kNewtonTolerance = 1e-10;

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

  // Arc length of the closest point on the polyline, or nullopt beyond
  // max_dist.
  std::optional<double> NearestSegmentS(const Eigen::Vector2d& query,
                                        double max_dist) const {
    double best_dist2 = max_dist * max_dist;
    std::optional<double> best;
    for (int i = 0; i + 1 < size(); ++i) {
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

#endif  // NUWAY_COMMON_FRENET_HPP_
