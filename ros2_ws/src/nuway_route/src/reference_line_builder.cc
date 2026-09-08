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
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  std::uint32_t lane_id = 0;
  double left_bound = 0.0;
  double right_bound = 0.0;
  double speed_limit = 0.0;
};

bool IsLateral(const nuway_map::Lane& from, std::uint32_t to) {
  return from.left_neighbor == to || from.right_neighbor == to;
}

// Drivable extent on one side: half the lane plus every consecutive
// same-direction drivable neighbor on that side.
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
// samples line up index by index) over the first blend_length_m of `to`.
void AppendBlend(const nuway_map::LaneGraph& graph, const nuway_map::Lane& from,
                 const nuway_map::Lane& to, double blend_length_m,
                 std::vector<RawPoint>* raw) {
  const std::size_t n = std::min(from.centerline.size(), to.centerline.size());
  double s = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    if (i > 0) {
      s += (to.centerline[i] - to.centerline[i - 1]).norm();
    }
    const double q = blend_length_m > 0.0
                         ? QuinticBlend(std::min(1.0, s / blend_length_m))
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

double QuinticBlend(double t) {
  const double u = std::max(0.0, std::min(1.0, t));
  return (10.0 * u * u * u) - (15.0 * u * u * u * u) +
         (6.0 * u * u * u * u * u);
}

std::optional<nuway_msgs::msg::ReferenceLine> BuildReferenceLine(
    const nuway_map::LaneGraph& graph,
    const std::vector<std::uint32_t>& lane_ids,
    const ReferenceLineOptions& options) {
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
    // blend from lane_i onto the run's last lane.
    std::size_t j = i;
    while (j + 1 < lane_ids.size() &&
           IsLateral(*graph.lane(lane_ids[j]), lane_ids[j + 1])) {
      ++j;
    }
    if (j == i) {
      AppendLane(graph, lane, &raw);
    } else {
      AppendBlend(graph, lane, *graph.lane(lane_ids[j]), options.blend_length_m,
                  &raw);
    }
    i = j + 1;
  }
  // 2. Extension past the goal along successors.
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
  std::size_t seg = 0;
  for (int k = 0; k < n; ++k) {
    const double s = std::min(total, k * options.spacing_m);
    while (seg + 2 < raw.size() && raw_s[seg + 1] <= s) {
      ++seg;
    }
    const double seg_len = raw_s[seg + 1] - raw_s[seg];
    const double t = seg_len > 1e-9 ? (s - raw_s[seg]) / seg_len : 0.0;
    const Eigen::Vector3d p = ((1.0 - t) * raw[seg].p) + (t * raw[seg + 1].p);
    const RawPoint& attr = t < 0.5 ? raw[seg] : raw[seg + 1];
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
