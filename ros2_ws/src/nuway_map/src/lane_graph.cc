#include "nuway_map/lane_graph.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <set>
#include <unordered_set>
#include <utility>

#include <nanoflann.hpp>

#include <nuway_common/frames.h>
#include <nuway_common/geometry.h>

namespace nuway_map {
namespace {

using Vector2dList = nuway_common::Vector2dList;

constexpr int kLaneIdOffset = 32;
constexpr int kLanesPerSection = 64;
constexpr int kSectionsPerRoad = 64;
constexpr std::uint32_t kTrafficLightHighBit = 0x80000000U;
// Longitudinal slack past a lane end that still counts as "on the lane".
constexpr double kEndTolerance = 0.2;

// Cumulative lateral position of the centre of `lane` in `section` at road
// arc length s; positive to the left of the reference line.
double LaneCenterT(const OdrRoad& road, const OdrLaneSection& section,
                   const OdrLane& lane, double s) {
  const double ds = s - section.s;
  double t = EvalPiecewise(road.lane_offsets, s);
  const std::vector<OdrLane>& side = lane.id > 0 ? section.left : section.right;
  const double sign = lane.id > 0 ? 1.0 : -1.0;
  for (const OdrLane& other : side) {
    const double width = EvalPiecewise(other.widths, ds);
    if (other.id == lane.id) {
      t += sign * 0.5 * width;
      break;
    }
    t += sign * width;
  }
  return t;
}

double LaneWidthAt(const OdrLaneSection& section, const OdrLane& lane,
                   double s) {
  return EvalPiecewise(lane.widths, s - section.s);
}

std::optional<LaneType> ParseLaneType(const std::string& type) {
  if (type == "driving") {
    return LaneType::kDriving;
  }
  if (type == "shoulder") {
    return LaneType::kShoulder;
  }
  if (type == "parking") {
    return LaneType::kParking;
  }
  if (type == "bidirectional") {
    return LaneType::kBidirectional;
  }
  return std::nullopt;
}

bool IsDrivable(LaneType type) {
  return type == LaneType::kDriving || type == LaneType::kBidirectional;
}

// The road mark on the outer side of the lane (or the centre mark for 0):
// the first record, as CARLA emits one mark per lane.
LaneChange OuterMark(const OdrLaneSection& section, int lane_id_odr) {
  if (lane_id_odr == 0) {
    return LaneChange::kNone;  // lane 0 separates opposite directions
  }
  const OdrLane* lane = section.Find(lane_id_odr);
  if (lane == nullptr || lane->marks.empty()) {
    return LaneChange::kNone;
  }
  return lane->marks.front().lane_change;
}

bool AllowsIncrease(LaneChange change) {
  return change == LaneChange::kBoth || change == LaneChange::kIncrease;
}

bool AllowsDecrease(LaneChange change) {
  return change == LaneChange::kBoth || change == LaneChange::kDecrease;
}

std::uint32_t Fnv1a(std::uint32_t seed, std::uint32_t value) {
  std::uint32_t hash = seed;
  for (int i = 0; i < 4; ++i) {
    hash ^= (value >> (8 * i)) & 0xFFU;
    hash *= 16777619U;
  }
  return hash;
}

int LaneCount(int road_id, const OpenDriveMap& map) {
  const auto it = map.roads.find(road_id);
  return it == map.roads.end() ? 0
                               : static_cast<int>(it->second.sections.size());
}

// Lane id in `road` reached through a road link (contact start = first
// section, contact end = last section).
std::optional<std::uint32_t> LinkedLaneId(const OpenDriveMap& map,
                                          const RoadLink& link,
                                          int lane_id_odr) {
  if (link.element != LinkElement::kRoad || lane_id_odr == kNoLink) {
    return std::nullopt;
  }
  const int sections = LaneCount(link.id, map);
  if (sections == 0) {
    return std::nullopt;
  }
  const int section_idx = link.contact_start ? 0 : sections - 1;
  return MakeLaneId(link.id, section_idx, lane_id_odr);
}

// Lanes reached through the junction at `link` from lane `lane_id_odr` of
// road `road_id`.
std::vector<std::uint32_t> JunctionLaneIds(const OpenDriveMap& map,
                                           const RoadLink& link, int road_id,
                                           int lane_id_odr) {
  std::vector<std::uint32_t> out;
  if (link.element != LinkElement::kJunction) {
    return out;
  }
  const auto junction = map.junctions.find(link.id);
  if (junction == map.junctions.end()) {
    return out;
  }
  for (const OdrConnection& conn : junction->second.connections) {
    if (conn.incoming_road != road_id) {
      continue;
    }
    const int sections = LaneCount(conn.connecting_road, map);
    if (sections == 0) {
      continue;
    }
    const int section_idx = conn.contact_start ? 0 : sections - 1;
    for (const OdrLaneLink& lane_link : conn.lane_links) {
      if (lane_link.from == lane_id_odr) {
        out.push_back(
            MakeLaneId(conn.connecting_road, section_idx, lane_link.to));
      }
    }
  }
  return out;
}

void PushUnique(std::vector<std::uint32_t>* vec, std::uint32_t value) {
  if (std::find(vec->begin(), vec->end(), value) == vec->end()) {
    vec->push_back(value);
  }
}

struct SignalSite {
  int road_id = -1;
  double s = 0.0;
  double t = 0.0;
  bool orientation_plus = true;
  std::vector<OdrValidity> validity;
};

bool PointInPolygon(const Eigen::Vector2d& p, const Vector2dList& poly) {
  bool inside = false;
  const std::size_t n = poly.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const Eigen::Vector2d& a = poly[i];
    const Eigen::Vector2d& b = poly[j];
    if ((a.y() > p.y()) != (b.y() > p.y())) {
      const double x =
          ((b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y())) + a.x();
      if (p.x() < x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

geometry_msgs::msg::Point ToPointMsg(const Eigen::Vector3d& p) {
  geometry_msgs::msg::Point out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}

geometry_msgs::msg::Polygon ToPolygonMsg(const Vector2dList& poly) {
  geometry_msgs::msg::Polygon out;
  out.points.reserve(poly.size());
  for (const Eigen::Vector2d& p : poly) {
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<float>(p.x());
    point.y = static_cast<float>(p.y());
    point.z = 0.0F;
    out.points.push_back(point);
  }
  return out;
}

Vector2dList FromPolygonMsg(const geometry_msgs::msg::Polygon& poly) {
  Vector2dList out;
  out.reserve(poly.points.size());
  for (const geometry_msgs::msg::Point32& p : poly.points) {
    out.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y));
  }
  return out;
}

}  // namespace

// KD-tree over every centerline sample of every lane.
struct LaneGraph::Index {
  struct Cloud {
    Vector2dList pts;
    std::vector<std::uint32_t> lane_of_point;
    std::vector<std::size_t> point_in_lane;
    // NOLINTBEGIN(readability-identifier-naming): nanoflann's adaptor interface
    std::size_t kdtree_get_point_count() const { return pts.size(); }
    double kdtree_get_pt(std::size_t idx, std::size_t dim) const {
      return dim == 0 ? pts[idx].x() : pts[idx].y();
    }
    template <class Bbox>
    bool kdtree_get_bbox(Bbox& /*bbox*/) const {
      return false;
    }
    // NOLINTEND(readability-identifier-naming)
  };
  using Tree = nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<double, Cloud>, Cloud, 2, std::size_t>;

  Cloud cloud;
  std::unique_ptr<Tree> tree;
  std::vector<nuway_common::ReferenceLine> reference_lines;  // by lane index
};

std::uint32_t MakeLaneId(int road_id, int section_idx, int lane_id_odr) {
  const int lane_slot =
      std::max(0, std::min(kLanesPerSection - 1, lane_id_odr + kLaneIdOffset));
  const int section_slot =
      std::max(0, std::min(kSectionsPerRoad - 1, section_idx));
  return (static_cast<std::uint32_t>(road_id) * kLanesPerSection *
          kSectionsPerRoad) +
         (static_cast<std::uint32_t>(section_slot) * kLanesPerSection) +
         static_cast<std::uint32_t>(lane_slot) + 1U;
}

std::uint32_t MakeSignalId(int road_id, int signal_id) {
  std::uint32_t hash = Fnv1a(2166136261U, static_cast<std::uint32_t>(road_id));
  hash = Fnv1a(hash, static_cast<std::uint32_t>(signal_id));
  hash &= ~kTrafficLightHighBit;  // high bit marks unmapped lights (M0 §2.2)
  return hash == 0U ? 1U : hash;
}

LaneGraph::LaneGraph() = default;
LaneGraph::~LaneGraph() = default;
LaneGraph::LaneGraph(LaneGraph&&) noexcept = default;
LaneGraph& LaneGraph::operator=(LaneGraph&&) noexcept = default;

LaneGraph LaneGraph::Build(const OpenDriveMap& map,
                           const LaneGraphOptions& options) {
  LaneGraph graph;
  graph.options_ = options;
  graph.geo_reference_ = map.geo_reference;

  // 1. Lanes with centerlines.
  for (const auto& [road_id, road] : map.roads) {
    for (std::size_t section_idx = 0; section_idx < road.sections.size();
         ++section_idx) {
      const OdrLaneSection& section = road.sections[section_idx];
      const double s0 = section.s;
      const double s1 = section.s_end;
      if (s1 - s0 < 1e-3) {
        continue;
      }
      const int n = std::max(2, static_cast<int>(std::ceil(
                                    (s1 - s0) / options.centerline_spacing_m)) +
                                    1);
      std::vector<const OdrLane*> odr_lanes;
      odr_lanes.reserve(section.left.size() + section.right.size());
      for (const OdrLane& lane : section.left) {
        odr_lanes.push_back(&lane);
      }
      for (const OdrLane& lane : section.right) {
        odr_lanes.push_back(&lane);
      }
      for (const OdrLane* odr_lane : odr_lanes) {
        const std::optional<LaneType> type = ParseLaneType(odr_lane->type);
        if (!type.has_value() || odr_lane->id == 0) {
          continue;
        }
        Lane lane;
        lane.id =
            MakeLaneId(road_id, static_cast<int>(section_idx), odr_lane->id);
        lane.road_id = static_cast<std::uint32_t>(road_id);
        lane.section_idx = static_cast<int>(section_idx);
        lane.lane_id_odr = odr_lane->id;
        lane.type = *type;
        lane.section_s_begin = s0;
        lane.section_s_end = s1;
        lane.speed_limit_mps = road.speed_limit_mps > 0.0
                                   ? road.speed_limit_mps
                                   : options.default_speed_limit_mps;
        lane.centerline.reserve(static_cast<std::size_t>(n));
        lane.width.reserve(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
          const double s = s0 + ((s1 - s0) * i / (n - 1));
          // The section is [s0, s1): the reference line is continuous across
          // the boundary but offsets and widths are not, so evaluate those
          // just inside the section for the last sample.
          const double s_lane = std::min(s, s1 - 1e-6);
          const double t = LaneCenterT(road, section, *odr_lane, s_lane);
          lane.centerline.push_back(road.AtLateral(s, t));
          lane.width.push_back(LaneWidthAt(section, *odr_lane, s_lane));
        }
        if (odr_lane->id > 0) {  // left lanes travel towards decreasing s
          std::reverse(lane.centerline.begin(), lane.centerline.end());
          std::reverse(lane.width.begin(), lane.width.end());
        }
        for (std::size_t i = 1; i < lane.centerline.size(); ++i) {
          lane.length_m += (lane.centerline[i] - lane.centerline[i - 1]).norm();
        }
        graph.index_by_id_[lane.id] = graph.lanes_.size();
        graph.lanes_.push_back(std::move(lane));
      }
    }
  }

  // 2. Topology (in the driving direction) and lateral relations.
  for (Lane& lane : graph.lanes_) {
    const OdrRoad& road = map.roads.at(static_cast<int>(lane.road_id));
    const OdrLaneSection& section =
        road.sections[static_cast<std::size_t>(lane.section_idx)];
    const OdrLane* odr_lane = section.Find(lane.lane_id_odr);
    if (odr_lane == nullptr) {
      continue;
    }
    const bool forward = lane.lane_id_odr < 0;  // travels towards +s
    const int last_section = static_cast<int>(road.sections.size()) - 1;

    // Successors in the driving direction.
    const int ahead_link =
        forward ? odr_lane->successor : odr_lane->predecessor;
    const bool at_road_end =
        forward ? lane.section_idx == last_section : lane.section_idx == 0;
    if (!at_road_end) {
      if (ahead_link != kNoLink) {
        PushUnique(
            &lane.successors,
            MakeLaneId(static_cast<int>(lane.road_id),
                       lane.section_idx + (forward ? 1 : -1), ahead_link));
      }
    } else {
      const RoadLink& link = forward ? road.successor : road.predecessor;
      if (const std::optional<std::uint32_t> id =
              LinkedLaneId(map, link, ahead_link)) {
        PushUnique(&lane.successors, *id);
      }
      for (const std::uint32_t id : JunctionLaneIds(
               map, link, static_cast<int>(lane.road_id), lane.lane_id_odr)) {
        PushUnique(&lane.successors, id);
      }
    }
    // Predecessors in the driving direction.
    const int behind_link =
        forward ? odr_lane->predecessor : odr_lane->successor;
    const bool at_road_start =
        forward ? lane.section_idx == 0 : lane.section_idx == last_section;
    if (!at_road_start) {
      if (behind_link != kNoLink) {
        PushUnique(
            &lane.predecessors,
            MakeLaneId(static_cast<int>(lane.road_id),
                       lane.section_idx + (forward ? -1 : 1), behind_link));
      }
    } else {
      const RoadLink& link = forward ? road.predecessor : road.successor;
      if (const std::optional<std::uint32_t> id =
              LinkedLaneId(map, link, behind_link)) {
        PushUnique(&lane.predecessors, *id);
      }
    }

    // Neighbors: adjacent same-direction drivable lanes; never across lane 0.
    const int k = std::abs(lane.lane_id_odr);
    const int sign = lane.lane_id_odr > 0 ? 1 : -1;
    // In the driving frame the reference line is on the left of right lanes
    // (-k) and on the right of left lanes (+k).
    const int inner = (k - 1) * sign;  // towards the reference line
    const int outer = (k + 1) * sign;  // away from it
    const auto neighbor_id = [&](int lane_id_odr) -> std::uint32_t {
      if (lane_id_odr == 0) {
        return 0;
      }
      const std::uint32_t id = MakeLaneId(static_cast<int>(lane.road_id),
                                          lane.section_idx, lane_id_odr);
      const Lane* other = graph.lane(id);
      return (other != nullptr && IsDrivable(other->type)) ? id : 0;
    };
    const std::uint32_t inner_id = neighbor_id(inner);
    const std::uint32_t outer_id = neighbor_id(outer);
    const LaneChange own_mark = OuterMark(section, lane.lane_id_odr);
    const LaneChange inner_mark = OuterMark(section, inner);
    if (forward) {
      lane.left_neighbor = inner_id;
      lane.right_neighbor = outer_id;
      // -k -> -(k-1) increases the id across the inner mark; -k -> -(k+1)
      // decreases it across our own mark.
      lane.left_change_allowed = inner_id != 0 && AllowsIncrease(inner_mark);
      lane.right_change_allowed = outer_id != 0 && AllowsDecrease(own_mark);
    } else {
      lane.left_neighbor = outer_id;
      lane.right_neighbor = inner_id;
      lane.left_change_allowed = outer_id != 0 && AllowsIncrease(own_mark);
      lane.right_change_allowed = inner_id != 0 && AllowsDecrease(inner_mark);
    }
  }
  // Symmetrize links and drop dangling ones.
  for (Lane& lane : graph.lanes_) {
    const auto exists = [&](std::uint32_t id) {
      return graph.lane(id) != nullptr;
    };
    lane.successors.erase(
        std::remove_if(lane.successors.begin(), lane.successors.end(),
                       [&](std::uint32_t id) { return !exists(id); }),
        lane.successors.end());
    lane.predecessors.erase(
        std::remove_if(lane.predecessors.begin(), lane.predecessors.end(),
                       [&](std::uint32_t id) { return !exists(id); }),
        lane.predecessors.end());
  }
  for (std::size_t i = 0; i < graph.lanes_.size(); ++i) {
    const std::uint32_t id = graph.lanes_[i].id;
    for (const std::uint32_t succ :
         std::vector<std::uint32_t>(graph.lanes_[i].successors)) {
      PushUnique(&graph.lanes_[graph.index_by_id_.at(succ)].predecessors, id);
    }
    for (const std::uint32_t pred :
         std::vector<std::uint32_t>(graph.lanes_[i].predecessors)) {
      PushUnique(&graph.lanes_[graph.index_by_id_.at(pred)].successors, id);
    }
  }

  // 3. Signals: definitions keyed by OpenDRIVE signal id, sites from the
  // definition's own validity and every signalReference.
  std::map<int, const OdrSignal*> signal_defs;
  std::map<int, int> signal_road;
  std::map<int, std::vector<SignalSite>> sites;
  for (const auto& [road_id, road] : map.roads) {
    for (const OdrSignal& signal : road.signals) {
      signal_defs[signal.id] = &signal;
      signal_road[signal.id] = road_id;
      SignalSite site;
      site.road_id = road_id;
      site.s = signal.s;
      site.t = signal.t;
      site.orientation_plus = signal.orientation_plus;
      site.validity = signal.validity;
      sites[signal.id].push_back(site);
    }
    for (const OdrSignalReference& ref : road.signal_references) {
      SignalSite site;
      site.road_id = road_id;
      site.s = ref.s;
      site.t = ref.t;
      site.orientation_plus = ref.orientation_plus;
      site.validity = ref.validity;
      sites[ref.id].push_back(site);
    }
  }
  std::unordered_set<std::uint32_t> used_signal_ids;
  for (const auto& [signal_id, signal] : signal_defs) {
    const bool is_light = signal->type == "1000001";
    const bool is_stop = signal->type == "206";
    if (!is_light && !is_stop) {
      continue;
    }
    std::uint32_t id = MakeSignalId(signal_road.at(signal_id), signal_id);
    while (used_signal_ids.count(id) != 0U) {
      id = (id + 1U) & ~kTrafficLightHighBit;
      id = id == 0U ? 1U : id;
    }
    used_signal_ids.insert(id);

    std::vector<std::uint32_t> affected;
    std::optional<Eigen::Vector3d> stop_line;
    std::optional<double> stop_heading;
    std::optional<double> stop_width;
    for (const SignalSite& site : sites[signal_id]) {
      const OdrRoad& road = map.roads.at(site.road_id);
      const OdrLaneSection& section =
          road.sections[static_cast<std::size_t>(road.SectionIndex(site.s))];
      for (const OdrValidity& validity : site.validity) {
        for (int lane_id_odr = validity.from_lane;
             lane_id_odr <= validity.to_lane; ++lane_id_odr) {
          if (lane_id_odr == 0) {
            continue;
          }
          const std::optional<std::uint32_t> lane_id =
              graph.LaneIdAt(site.road_id, lane_id_odr, site.s);
          if (!lane_id.has_value()) {
            continue;
          }
          PushUnique(&affected, *lane_id);
          if (!stop_line.has_value()) {
            const OdrLane* odr_lane = section.Find(lane_id_odr);
            if (odr_lane == nullptr) {
              continue;
            }
            const double t = LaneCenterT(road, section, *odr_lane, site.s);
            stop_line = road.AtLateral(site.s, t);
            const double road_hdg = road.At(site.s).hdg;
            // Driving direction of the governed lane; the signal faces it.
            const double travel =
                lane_id_odr < 0 ? road_hdg : road_hdg + nuway_common::kPi;
            stop_heading = nuway_common::WrapAngle(travel + nuway_common::kPi);
            stop_width = LaneWidthAt(section, *odr_lane, site.s);
          }
        }
      }
    }
    if (!stop_line.has_value()) {
      // No governed lane: place it at the signal itself.
      const OdrRoad& road = map.roads.at(signal_road.at(signal_id));
      stop_line = road.AtLateral(signal->s, signal->t);
      const double road_hdg = road.At(signal->s).hdg;
      stop_heading = nuway_common::WrapAngle(
          road_hdg + (signal->orientation_plus ? nuway_common::kPi : 0.0) +
          signal->h_offset);
      stop_width = 3.5;
    }
    if (is_light) {
      TrafficLightMapping light;
      light.id = id;
      light.affected_lane_ids = affected;
      light.stop_line = *stop_line;
      light.heading_rad = *stop_heading;
      graph.traffic_lights_.push_back(std::move(light));
    } else {
      StopSign sign;
      sign.id = id;
      sign.affected_lane_ids = affected;
      sign.stop_line = *stop_line;
      // Trigger volume: a lane-wide box from 2 m before to 2 m past the line.
      const double travel = *stop_heading + nuway_common::kPi;
      const Eigen::Vector2d fwd{std::cos(travel), std::sin(travel)};
      const Eigen::Vector2d left{-fwd.y(), fwd.x()};
      const Eigen::Vector2d c = stop_line->head<2>();
      const double half_w = 0.5 * *stop_width;
      sign.trigger_volume = {
          c - (2.0 * fwd) + (half_w * left), c + (2.0 * fwd) + (half_w * left),
          c + (2.0 * fwd) - (half_w * left), c - (2.0 * fwd) - (half_w * left)};
      graph.stop_signs_.push_back(std::move(sign));
    }
  }

  // 4. Crosswalks from objects with an outline (or length x width box).
  for (const auto& [road_id, road] : map.roads) {
    for (const OdrObject& object : road.objects) {
      if (object.type != "crosswalk") {
        continue;
      }
      Crosswalk crosswalk;
      crosswalk.id = MakeSignalId(road_id, object.id);
      const Eigen::Vector3d base = road.AtLateral(object.s, object.t);
      const double angle = road.At(object.s).hdg + object.hdg;
      const Eigen::Vector2d u_axis{std::cos(angle), std::sin(angle)};
      const Eigen::Vector2d v_axis{-u_axis.y(), u_axis.x()};
      Vector2dList local = object.outline;
      if (local.empty()) {
        const double hl = 0.5 * object.length;
        const double hw = 0.5 * object.width;
        local = {{-hl, -hw}, {hl, -hw}, {hl, hw}, {-hl, hw}};
      }
      for (const Eigen::Vector2d& corner : local) {
        crosswalk.footprint.emplace_back(
            base.head<2>() + (corner.x() * u_axis) + (corner.y() * v_axis));
      }
      if (crosswalk.footprint.size() < 3) {
        continue;
      }
      for (const Lane& lane : graph.lanes_) {
        for (const Eigen::Vector3d& p : lane.centerline) {
          if (PointInPolygon(p.head<2>(), crosswalk.footprint)) {
            crosswalk.crossing_lane_ids.push_back(lane.id);
            break;
          }
        }
      }
      graph.crosswalks_.push_back(std::move(crosswalk));
    }
  }

  graph.Finalize();
  return graph;
}

void LaneGraph::Finalize() {
  index_by_id_.clear();
  for (std::size_t i = 0; i < lanes_.size(); ++i) {
    index_by_id_[lanes_[i].id] = i;
  }
  index_ = std::make_unique<Index>();
  index_->reference_lines.reserve(lanes_.size());
  for (const Lane& lane : lanes_) {
    Vector2dList points;
    points.reserve(lane.centerline.size());
    for (std::size_t j = 0; j < lane.centerline.size(); ++j) {
      points.emplace_back(lane.centerline[j].head<2>());
      index_->cloud.pts.emplace_back(lane.centerline[j].head<2>());
      index_->cloud.lane_of_point.push_back(lane.id);
      index_->cloud.point_in_lane.push_back(j);
    }
    index_->reference_lines.push_back(
        nuway_common::ReferenceLine::FromPoints(points));
  }
  index_->tree = std::make_unique<Index::Tree>(
      2, index_->cloud, nanoflann::KDTreeSingleIndexAdaptorParams(16));
  index_->tree->buildIndex();
}

const Lane* LaneGraph::lane(std::uint32_t id) const {
  const auto it = index_by_id_.find(id);
  return it == index_by_id_.end() ? nullptr : &lanes_[it->second];
}

std::vector<std::uint32_t> LaneGraph::Successors(std::uint32_t id) const {
  const Lane* l = lane(id);
  return l == nullptr ? std::vector<std::uint32_t>{} : l->successors;
}

std::vector<std::uint32_t> LaneGraph::Predecessors(std::uint32_t id) const {
  const Lane* l = lane(id);
  return l == nullptr ? std::vector<std::uint32_t>{} : l->predecessors;
}

std::vector<std::uint32_t> LaneGraph::Neighbors(std::uint32_t id) const {
  std::vector<std::uint32_t> out;
  const Lane* l = lane(id);
  if (l == nullptr) {
    return out;
  }
  if (l->left_neighbor != 0) {
    out.push_back(l->left_neighbor);
  }
  if (l->right_neighbor != 0) {
    out.push_back(l->right_neighbor);
  }
  return out;
}

std::vector<const TrafficLightMapping*> LaneGraph::TrafficLightsForLane(
    std::uint32_t id) const {
  std::vector<const TrafficLightMapping*> out;
  for (const TrafficLightMapping& light : traffic_lights_) {
    if (std::find(light.affected_lane_ids.begin(),
                  light.affected_lane_ids.end(),
                  id) != light.affected_lane_ids.end()) {
      out.push_back(&light);
    }
  }
  return out;
}

std::optional<Eigen::Vector3d> LaneGraph::StopLineForLane(
    std::uint32_t id) const {
  for (const StopSign& sign : stop_signs_) {
    if (std::find(sign.affected_lane_ids.begin(), sign.affected_lane_ids.end(),
                  id) != sign.affected_lane_ids.end()) {
      return sign.stop_line;
    }
  }
  const std::vector<const TrafficLightMapping*> lights =
      TrafficLightsForLane(id);
  if (!lights.empty()) {
    return lights.front()->stop_line;
  }
  return std::nullopt;
}

std::optional<std::uint32_t> LaneGraph::LaneIdAt(int road_id, int lane_id_odr,
                                                 double road_s) const {
  // Roads have few sections, so scanning the section slots is cheap.
  const Lane* best = nullptr;
  for (int section_idx = 0; section_idx < kSectionsPerRoad; ++section_idx) {
    const Lane* candidate = lane(MakeLaneId(road_id, section_idx, lane_id_odr));
    if (candidate == nullptr) {
      continue;
    }
    if (best == nullptr || road_s >= candidate->section_s_begin) {
      best = candidate;
    }
  }
  if (best == nullptr) {
    return std::nullopt;
  }
  return best->id;
}

std::optional<std::uint32_t> LaneGraph::LaneIdNear(int road_id, int lane_id_odr,
                                                   double x, double y) const {
  const Eigen::Vector2d query{x, y};
  const Lane* best = nullptr;
  double best_dist = 0.0;
  for (int section_idx = 0; section_idx < kSectionsPerRoad; ++section_idx) {
    const Lane* candidate = lane(MakeLaneId(road_id, section_idx, lane_id_odr));
    if (candidate == nullptr) {
      continue;
    }
    for (const Eigen::Vector3d& p : candidate->centerline) {
      const double dist = (p.head<2>() - query).norm();
      if (best == nullptr || dist < best_dist) {
        best = candidate;
        best_dist = dist;
      }
    }
  }
  if (best == nullptr) {
    return std::nullopt;
  }
  return best->id;
}

const nuway_common::ReferenceLine* LaneGraph::reference_line(
    std::uint32_t id) const {
  const auto it = index_by_id_.find(id);
  if (it == index_by_id_.end() || index_ == nullptr) {
    return nullptr;
  }
  return &index_->reference_lines[it->second];
}

std::vector<LaneQuery> LaneGraph::LanesNear(double x, double y,
                                            double max_dist) const {
  std::vector<LaneQuery> out;
  if (index_ == nullptr || index_->cloud.pts.empty()) {
    return out;
  }
  // Samples are <= spacing apart, so a point within max_dist of the line is
  // within max_dist + spacing of a sample.
  const double radius = max_dist + options_.centerline_spacing_m;
  const double query[2] = {x, y};
  std::vector<nanoflann::ResultItem<std::size_t, double>> matches;
  nanoflann::SearchParameters params;
  params.sorted = true;
  index_->tree->radiusSearch(query, radius * radius, matches, params);
  std::set<std::uint32_t> candidates;
  for (const auto& match : matches) {
    candidates.insert(index_->cloud.lane_of_point[match.first]);
  }
  for (const std::uint32_t lane_id : candidates) {
    const nuway_common::ReferenceLine* line = reference_line(lane_id);
    if (line == nullptr) {
      continue;
    }
    const std::optional<nuway_common::FrenetPoint> frenet =
        line->ToFrenet(x, y, max_dist);
    if (!frenet.has_value()) {
      continue;
    }
    // ToFrenet clamps s to the line, so a point past either end projects
    // onto the end point with a longitudinal residual: reject those.
    const nuway_common::CartesianPoint foot = line->PointAt(frenet->s);
    const double along = ((x - foot.x) * std::cos(foot.heading)) +
                         ((y - foot.y) * std::sin(foot.heading));
    if (std::abs(along) > kEndTolerance) {
      continue;
    }
    out.push_back(LaneQuery{lane_id, frenet->s, frenet->d});
  }
  std::sort(out.begin(), out.end(), [](const LaneQuery& a, const LaneQuery& b) {
    return std::abs(a.d) < std::abs(b.d);
  });
  return out;
}

std::optional<LaneQuery> LaneGraph::NearestLane(double x, double y, double yaw,
                                                double max_dist) const {
  std::optional<LaneQuery> best;
  for (const LaneQuery& query : LanesNear(x, y, max_dist)) {
    const nuway_common::ReferenceLine* line = reference_line(query.lane_id);
    if (line == nullptr) {
      continue;
    }
    const double heading_err =
        std::abs(nuway_common::WrapAngle(line->HeadingAt(query.s) - yaw));
    if (heading_err > options_.heading_tolerance_rad) {
      continue;
    }
    if (!best.has_value() || std::abs(query.d) < std::abs(best->d)) {
      best = query;
    }
  }
  return best;
}

nuway_msgs::msg::LaneGraph LaneGraph::ToMsg() const {
  nuway_msgs::msg::LaneGraph msg;
  msg.header.frame_id = nuway_common::kFrameMap;
  msg.lanes.reserve(lanes_.size());
  for (const Lane& lane : lanes_) {
    nuway_msgs::msg::Lane out;
    out.id = lane.id;
    out.road_id = lane.road_id;
    out.lane_id_odr = lane.lane_id_odr;
    out.type = static_cast<std::uint8_t>(lane.type);
    out.centerline.reserve(lane.centerline.size());
    for (const Eigen::Vector3d& p : lane.centerline) {
      out.centerline.push_back(ToPointMsg(p));
    }
    out.width.reserve(lane.width.size());
    for (const double w : lane.width) {
      out.width.push_back(static_cast<float>(w));
    }
    out.successors = lane.successors;
    out.predecessors = lane.predecessors;
    out.left_neighbor = lane.left_neighbor;
    out.right_neighbor = lane.right_neighbor;
    out.left_change_allowed = lane.left_change_allowed;
    out.right_change_allowed = lane.right_change_allowed;
    out.speed_limit = static_cast<float>(lane.speed_limit_mps);
    msg.lanes.push_back(std::move(out));
  }
  for (const TrafficLightMapping& light : traffic_lights_) {
    nuway_msgs::msg::TrafficLightMapping out;
    out.id = light.id;
    out.affected_lane_ids = light.affected_lane_ids;
    out.stop_line = ToPointMsg(light.stop_line);
    out.heading = static_cast<float>(light.heading_rad);
    out.bulb_position = ToPointMsg(light.bulb_position);
    out.from_override = light.from_override;
    msg.traffic_lights.push_back(std::move(out));
  }
  for (const StopSign& sign : stop_signs_) {
    nuway_msgs::msg::StopSign out;
    out.id = sign.id;
    out.affected_lane_ids = sign.affected_lane_ids;
    out.stop_line = ToPointMsg(sign.stop_line);
    out.trigger_volume = ToPolygonMsg(sign.trigger_volume);
    msg.stop_signs.push_back(std::move(out));
  }
  for (const Crosswalk& crosswalk : crosswalks_) {
    nuway_msgs::msg::Crosswalk out;
    out.id = crosswalk.id;
    out.footprint = ToPolygonMsg(crosswalk.footprint);
    out.crossing_lane_ids = crosswalk.crossing_lane_ids;
    msg.crosswalks.push_back(std::move(out));
  }
  msg.has_georeference = geo_reference_.valid;
  msg.geo_lat0 = geo_reference_.lat0;
  msg.geo_lon0 = geo_reference_.lon0;
  msg.geo_alt0 = geo_reference_.alt0;
  return msg;
}

LaneGraph LaneGraph::FromMsg(const nuway_msgs::msg::LaneGraph& msg,
                             const LaneGraphOptions& options) {
  LaneGraph graph;
  graph.options_ = options;
  graph.lanes_.reserve(msg.lanes.size());
  for (const nuway_msgs::msg::Lane& in : msg.lanes) {
    Lane lane;
    lane.id = in.id;
    lane.road_id = in.road_id;
    lane.section_idx =
        static_cast<int>(((in.id - 1U) / kLanesPerSection) % kSectionsPerRoad);
    lane.lane_id_odr = in.lane_id_odr;
    lane.type = static_cast<LaneType>(in.type);
    lane.centerline.reserve(in.centerline.size());
    for (const geometry_msgs::msg::Point& p : in.centerline) {
      lane.centerline.emplace_back(p.x, p.y, p.z);
    }
    lane.width.assign(in.width.begin(), in.width.end());
    lane.successors = in.successors;
    lane.predecessors = in.predecessors;
    lane.left_neighbor = in.left_neighbor;
    lane.right_neighbor = in.right_neighbor;
    lane.left_change_allowed = in.left_change_allowed;
    lane.right_change_allowed = in.right_change_allowed;
    lane.speed_limit_mps = static_cast<double>(in.speed_limit);
    for (std::size_t i = 1; i < lane.centerline.size(); ++i) {
      lane.length_m += (lane.centerline[i] - lane.centerline[i - 1]).norm();
    }
    graph.lanes_.push_back(std::move(lane));
  }
  for (const nuway_msgs::msg::TrafficLightMapping& in : msg.traffic_lights) {
    TrafficLightMapping light;
    light.id = in.id;
    light.affected_lane_ids = in.affected_lane_ids;
    light.stop_line =
        Eigen::Vector3d{in.stop_line.x, in.stop_line.y, in.stop_line.z};
    light.heading_rad = static_cast<double>(in.heading);
    light.bulb_position = Eigen::Vector3d{
        in.bulb_position.x, in.bulb_position.y, in.bulb_position.z};
    light.from_override = in.from_override;
    graph.traffic_lights_.push_back(std::move(light));
  }
  for (const nuway_msgs::msg::StopSign& in : msg.stop_signs) {
    StopSign sign;
    sign.id = in.id;
    sign.affected_lane_ids = in.affected_lane_ids;
    sign.stop_line =
        Eigen::Vector3d{in.stop_line.x, in.stop_line.y, in.stop_line.z};
    sign.trigger_volume = FromPolygonMsg(in.trigger_volume);
    graph.stop_signs_.push_back(std::move(sign));
  }
  for (const nuway_msgs::msg::Crosswalk& in : msg.crosswalks) {
    Crosswalk crosswalk;
    crosswalk.id = in.id;
    crosswalk.footprint = FromPolygonMsg(in.footprint);
    crosswalk.crossing_lane_ids = in.crossing_lane_ids;
    graph.crosswalks_.push_back(std::move(crosswalk));
  }
  graph.geo_reference_.valid = msg.has_georeference;
  graph.geo_reference_.lat0 = msg.geo_lat0;
  graph.geo_reference_.lon0 = msg.geo_lon0;
  graph.geo_reference_.alt0 = msg.geo_alt0;
  graph.Finalize();
  return graph;
}

}  // namespace nuway_map
