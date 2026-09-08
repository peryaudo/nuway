// Reference line construction behind reference_line_builder.h. The steps
// (numbered in BuildReferenceLine) and the ideas behind them:
//
// Concatenation. Lane centerlines are sampled at 1 m (nuway_map) and each
// lane's last sample coincides with its successor's first, so joining them
// drops the duplicate. Every raw sample also carries the attributes the
// message needs (lane id, bounds, speed limit).
//
// Lateral blend. At a lane change the line must move one lane over without
// a kink. Neighbouring lanes of one road section share the sample grid
// (same s per index), so the blended sample i is the mix
//   p_i = (1 - q) * from_i + q * to_i,  q = QuinticBlend((s_i - s0) / L),
// which slides from the source to the target over the length L starting at
// s0. QuinticBlend(t) = 10 t^3 - 15 t^4 + 6 t^5 is the lowest-degree
// polynomial with q(0) = 0, q(1) = 1 and zero first *and* second
// derivatives at both ends: q' = 30 t^2 (1 - t)^2 and
// q'' = 60 t (1 - t)(1 - 2 t) both vanish at t = 0 and t = 1. Along the
// blend the lateral offset is therefore C2 in s, so the line's heading
// (first derivative) and curvature (second derivative) are continuous where
// the blend starts and ends, and the controller sees no steering step. A
// linear or cubic (smoothstep) ramp would leave a curvature jump.
//
// Resampling. Consumers index the line by arc length at a fixed spacing.
// The raw polyline is walked once with a segment cursor; each output s lands
// on a segment and position, heading and curvature are interpolated inside
// it (heading through WrapAngle so the +-pi seam does not average to 0).
//
// Heading and curvature. Heading is the central difference direction
// atan2(p[i+1] - p[i-1]) (one-sided at the ends). Curvature is Menger's:
// for three consecutive points a, b, c it is 1/R of their circumscribed
// circle, 4 * area(abc) / (|ab| |bc| |ca|), with the sign of the cross
// product (positive for a left turn). On 1 m samples the sampling noise is
// visible, so the values are smoothed with a short moving average, and both
// are computed on the raw samples (which lie on the true lane geometry)
// before resampling; on the resampled polyline the triples would alternate
// between raw vertices and chord midpoints (+-15 % on a 30 m circle,
// docs/milestones/M0_bringup.md Decisions log, task 8).
//
// Extension. The line continues past the goal along successors so a
// planner or controller looking ahead near the goal never runs out of line.
#include "nuway_route/reference_line_builder.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <nuway_common/frames.h>
#include <nuway_common/frenet.h>
#include <nuway_common/geometry.h>

namespace nuway_route {
namespace {

// One raw centerline sample with the attributes the message carries.
struct RawPoint {
  Eigen::Vector3d p = Eigen::Vector3d::Zero();  // map frame, metres
  std::uint32_t lane_id = 0;
  double left_bound = 0.0;   // drivable extent to the left, metres
  double right_bound = 0.0;  // drivable extent to the right, metres
  double speed_limit = 0.0;  // m/s
};

// True when `to` is a lateral neighbour of `from` (a lane-change edge).
bool IsLateral(const nuway_map::Lane& from, std::uint32_t to) {
  return from.left_neighbor == to || from.right_neighbor == to;
}

// Drivable extent on one side at sample `i`: half the lane plus every
// consecutive same-direction drivable neighbor on that side (neighbours are
// same-direction by construction, nuway_map). The guard bounds the walk on
// a malformed neighbour cycle; the width index is clamped for lanes whose
// width vector is shorter than the centerline.
double SideBound(const nuway_map::LaneGraph& graph, const nuway_map::Lane& lane,
                 std::size_t i, bool left) {
  double bound = 0.5 * lane.width[std::min(i, lane.width.size() - 1)];
  std::uint32_t next = left ? lane.left_neighbor : lane.right_neighbor;
  int guard = 0;
  while (next != 0 && guard++ < 8) {
    const nuway_map::Lane* neighbor = graph.lane(next);
    if (neighbor == nullptr || neighbor->width.empty()) {
      break;
    }
    bound += neighbor->width[std::min(i, neighbor->width.size() - 1)];
    next = left ? neighbor->left_neighbor : neighbor->right_neighbor;
  }
  return bound;
}

// Raw sample at position `p` carrying the attributes of `lane` at index `i`.
RawPoint MakeRaw(const nuway_map::LaneGraph& graph, const nuway_map::Lane& lane,
                 std::size_t i, const Eigen::Vector3d& p) {
  RawPoint raw;
  raw.p = p;
  raw.lane_id = lane.id;
  raw.left_bound = SideBound(graph, lane, i, true);
  raw.right_bound = SideBound(graph, lane, i, false);
  raw.speed_limit = lane.speed_limit_mps;
  return raw;
}

// Appends the lane's centerline, skipping a first sample that coincides with
// the previous lane's last (successive lanes share their boundary point).
void AppendLane(const nuway_map::LaneGraph& graph, const nuway_map::Lane& lane,
                std::vector<RawPoint>* raw) {
  for (std::size_t i = 0; i < lane.centerline.size(); ++i) {
    const Eigen::Vector3d& p = lane.centerline[i];
    if (!raw->empty() && (raw->back().p - p).norm() < 1e-3) {
      continue;  // shared boundary point
    }
    raw->push_back(MakeRaw(graph, lane, i, p));
  }
}

// Appends the lateral blend from `from` onto `to` (same section, so their
// samples line up index by index), starting at arc length `begin_s` along
// `to` (the ego's position on the first lane of the plan, else 0) and
// lasting min(blend_length_m, what is left of the section): a blend anchored
// at the section start put the line fully on the neighbour behind an ego
// already mid-section (an immediate 3.5 m error), and a fixed 30 m on a 15 m
// section ended with the line 79 % on the source lane and a sideways step
// at the section boundary. The blend parameter is `to`'s own arc length
// (accumulated from its samples) so the ease runs in metres of road; before
// `begin_s` the mix is 0 (still on `from`), after `begin_s` + length it is
// 1 (fully on `to`), and the length is at least 1 m so the division is
// safe. Samples of `to` beyond the shared index range are copied as is; the
// attributes of every blended sample are those of the target lane.
void AppendBlend(const nuway_map::LaneGraph& graph, const nuway_map::Lane& from,
                 const nuway_map::Lane& to, double blend_length_m,
                 double begin_s, std::vector<RawPoint>* raw) {
  const std::size_t n = std::min(from.centerline.size(), to.centerline.size());
  const double length = std::max(
      1.0, std::min(blend_length_m, to.length_m - std::max(0.0, begin_s)));
  double s = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    if (i > 0) {
      s += (to.centerline[i] - to.centerline[i - 1]).norm();
    }
    const double q =
        blend_length_m > 0.0
            ? QuinticBlend(std::max(0.0, std::min(1.0, (s - begin_s) / length)))
            : 1.0;
    const Eigen::Vector3d p =
        ((1.0 - q) * from.centerline[i]) + (q * to.centerline[i]);
    if (!raw->empty() && (raw->back().p - p).norm() < 1e-3) {
      continue;
    }
    raw->push_back(MakeRaw(graph, to, i, p));
  }
  for (std::size_t i = n; i < to.centerline.size(); ++i) {
    raw->push_back(MakeRaw(graph, to, i, to.centerline[i]));
  }
}

// Centered moving average of width `window` (half on each side); the
// window is truncated at the ends of the series, so the first and last
// values average over fewer samples rather than being padded.
std::vector<double> MovingAverage(const std::vector<double>& in, int window) {
  const int half = std::max(0, window / 2);
  std::vector<double> out(in.size(), 0.0);
  const int n = static_cast<int>(in.size());
  for (int i = 0; i < n; ++i) {
    double sum = 0.0;
    int count = 0;
    for (int j = std::max(0, i - half); j <= std::min(n - 1, i + half); ++j) {
      sum += in[static_cast<std::size_t>(j)];
      ++count;
    }
    out[static_cast<std::size_t>(i)] = count > 0 ? sum / count : 0.0;
  }
  return out;
}

}  // namespace

// 10 t^3 - 15 t^4 + 6 t^5 on t clamped to [0, 1]; see the file header for
// why its first and second derivatives vanish at both ends.
double QuinticBlend(double t) {
  const double u = std::max(0.0, std::min(1.0, t));
  return (10.0 * u * u * u) - (15.0 * u * u * u * u) +
         (6.0 * u * u * u * u * u);
}

std::optional<nuway_msgs::msg::ReferenceLine> BuildReferenceLine(
    const nuway_map::LaneGraph& graph,
    const std::vector<std::uint32_t>& lane_ids,
    const ReferenceLineOptions& options, double start_s_m) {
  if (lane_ids.empty()) {
    return std::nullopt;
  }
  for (const std::uint32_t id : lane_ids) {
    if (graph.lane(id) == nullptr) {
      return std::nullopt;
    }
  }
  // 1. Raw polyline along the route.
  std::vector<RawPoint> raw;
  std::size_t i = 0;
  while (i < lane_ids.size()) {
    const nuway_map::Lane& lane = *graph.lane(lane_ids[i]);
    // A run of lateral edges lane_i -> lane_{i+1} -> ... collapses into one
    // blend from lane_i onto the run's last lane (a double lane change is
    // one sideways move of two lane widths, not two blends in sequence).
    std::size_t j = i;
    while (j + 1 < lane_ids.size() &&
           IsLateral(*graph.lane(lane_ids[j]), lane_ids[j + 1])) {
      ++j;
    }
    if (j == i) {
      AppendLane(graph, lane, &raw);
    } else {
      AppendBlend(graph, lane, *graph.lane(lane_ids[j]), options.blend_length_m,
                  i == 0 ? start_s_m : 0.0, &raw);
    }
    i = j + 1;
  }
  // 2. Extension past the goal along successors: the first successor at
  // each step, whole lanes until extension_m is covered; the guard bounds
  // the walk on a short ring.
  {
    const nuway_map::Lane* last = graph.lane(lane_ids.back());
    double extended = 0.0;
    int guard = 0;
    while (extended < options.extension_m && !last->successors.empty() &&
           guard++ < 64) {
      const nuway_map::Lane* next = graph.lane(last->successors.front());
      if (next == nullptr) {
        break;
      }
      AppendLane(graph, *next, &raw);
      extended += next->length_m;
      last = next;
    }
  }
  if (raw.size() < 2) {
    return std::nullopt;
  }
  // 3. Heading (central difference) and Menger curvature on the raw points,
  // which lie on the true lane geometry; the curvature is smoothed with the
  // moving average there. Computing them after resampling would alternate
  // between vertices and chord midpoints of the raw polyline.
  nuway_common::Vector2dList raw_xy;
  raw_xy.reserve(raw.size());
  for (const RawPoint& r : raw) {
    raw_xy.emplace_back(r.p.x(), r.p.y());
  }
  const nuway_common::ReferenceLine raw_line =
      nuway_common::ReferenceLine::FromPoints(raw_xy);
  const std::vector<double>& raw_s = raw_line.s();
  const std::vector<double>& raw_heading = raw_line.heading();
  const std::vector<double> raw_curvature =
      MovingAverage(raw_line.curvature(), options.curvature_window);
  // 4. Resample at spacing_m by linear interpolation along the polyline;
  // discrete attributes come from the nearer raw point.
  const double total = raw_s.back();
  int n = static_cast<int>(std::floor(total / options.spacing_m)) + 1;
  // Always end on the last raw point (goal or extension end); the final gap
  // is then shorter than spacing_m.
  const bool append_end = total - ((n - 1) * options.spacing_m) > 1e-6;
  n += append_end ? 1 : 0;
  nuway_msgs::msg::ReferenceLine msg;
  msg.header.frame_id = nuway_common::kFrameMap;
  // `seg` is the raw segment [seg, seg + 1] holding the current s; output s
  // is increasing, so the cursor only ever moves forward.
  std::size_t seg = 0;
  for (int k = 0; k < n; ++k) {
    const double s = std::min(total, k * options.spacing_m);
    while (seg + 2 < raw.size() && raw_s[seg + 1] <= s) {
      ++seg;
    }
    // t is the fraction of the segment covered; a degenerate (zero-length)
    // segment takes its first point.
    const double seg_len = raw_s[seg + 1] - raw_s[seg];
    const double t = seg_len > 1e-9 ? (s - raw_s[seg]) / seg_len : 0.0;
    const Eigen::Vector3d p = ((1.0 - t) * raw[seg].p) + (t * raw[seg + 1].p);
    const RawPoint& attr = t < 0.5 ? raw[seg] : raw[seg + 1];
    // Interpolate the heading through the wrapped difference so a segment
    // crossing the +-pi seam turns the short way.
    const double heading = nuway_common::WrapAngle(
        raw_heading[seg] +
        (t * nuway_common::WrapAngle(raw_heading[seg + 1] - raw_heading[seg])));
    const double curvature =
        ((1.0 - t) * raw_curvature[seg]) + (t * raw_curvature[seg + 1]);
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    msg.points.push_back(point);
    msg.s.push_back(static_cast<float>(s));
    msg.heading.push_back(static_cast<float>(heading));
    msg.curvature.push_back(static_cast<float>(curvature));
    msg.lane_id.push_back(attr.lane_id);
    msg.left_bound.push_back(static_cast<float>(attr.left_bound));
    msg.right_bound.push_back(static_cast<float>(attr.right_bound));
    msg.speed_limit.push_back(static_cast<float>(attr.speed_limit));
  }
  return msg;
}

}  // namespace nuway_route
