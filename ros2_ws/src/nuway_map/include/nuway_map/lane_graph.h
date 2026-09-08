// LaneGraph (M0): lanes with ids per (road, section, lane), topology through
// junctions, neighbors and change flags, speed limits, traffic-light / stop
// sign / crosswalk associations, and the NearestLane query (KD-tree over
// centerline samples + heading check). Built from an OpenDriveMap; converts to
// and from nuway_msgs/LaneGraph so runtime nodes can rebuild it from the
// latched topic (docs/milestones/M0_bringup.md §2.4). No rclcpp.
#ifndef NUWAY_MAP_LANE_GRAPH_H_
#define NUWAY_MAP_LANE_GRAPH_H_

#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include <nuway_common/frenet.h>
#include <nuway_common/geometry.h>
#include <nuway_msgs/msg/lane_graph.hpp>

#include "nuway_map/opendrive_parser.h"

namespace nuway_map {

// Lane types kept from OpenDRIVE (msg Lane.TYPE_*).
enum class LaneType : std::uint8_t {
  kDriving = 0,
  kShoulder = 1,
  kParking = 2,
  kBidirectional = 3,
  kOther = 4,
};

struct Lane {
  std::uint32_t id = 0;
  std::uint32_t road_id = 0;
  int section_idx = 0;
  int lane_id_odr = 0;
  LaneType type = LaneType::kDriving;
  nuway_common::Vector3dList centerline;  // driving direction, <= 2 m spacing
  std::vector<double> width;              // per point
  std::vector<std::uint32_t> successors;
  std::vector<std::uint32_t> predecessors;
  std::uint32_t left_neighbor = 0;  // 0 = none
  std::uint32_t right_neighbor = 0;
  bool left_change_allowed = false;
  bool right_change_allowed = false;
  double speed_limit_mps = 0.0;  // 0 = unknown
  double length_m = 0.0;
  // Road arc-length range of the lane section; known only for graphs built
  // from a map (FromMsg leaves the range open).
  double section_s_begin = 0.0;
  double section_s_end = 1e300;
};

struct TrafficLightMapping {
  std::uint32_t id = 0;
  std::vector<std::uint32_t> affected_lane_ids;
  Eigen::Vector3d stop_line = Eigen::Vector3d::Zero();
  double heading_rad = 0.0;
  Eigen::Vector3d bulb_position = Eigen::Vector3d::Zero();
  bool from_override = false;
};

struct StopSign {
  std::uint32_t id = 0;
  std::vector<std::uint32_t> affected_lane_ids;
  Eigen::Vector3d stop_line = Eigen::Vector3d::Zero();
  nuway_common::Vector2dList trigger_volume;
};

struct Crosswalk {
  std::uint32_t id = 0;
  nuway_common::Vector2dList footprint;
  std::vector<std::uint32_t> crossing_lane_ids;
};

struct LaneQuery {
  std::uint32_t lane_id = 0;
  double s = 0.0;  // along the lane centerline, m
  double d = 0.0;  // lateral offset, positive left
};

struct LaneGraphOptions {
  double default_speed_limit_mps = 8.33;
  double centerline_spacing_m = 1.0;
  // Heading tolerance for NearestLane; lanes whose local heading differs by
  // more than this from the query yaw are skipped.
  double heading_tolerance_rad = 1.3;
};

// Stable lane id for (road, section, OpenDRIVE lane id); never 0.
std::uint32_t MakeLaneId(int road_id, int section_idx, int lane_id_odr);
// Stable signal id for (road, signal); never 0 (TrafficLightMapping.id).
std::uint32_t MakeSignalId(int road_id, int signal_id);

class LaneGraph {
 public:
  LaneGraph();
  ~LaneGraph();
  LaneGraph(LaneGraph&&) noexcept;
  LaneGraph& operator=(LaneGraph&&) noexcept;
  LaneGraph(const LaneGraph&) = delete;
  LaneGraph& operator=(const LaneGraph&) = delete;

  // Builds the graph from a parsed map.
  static LaneGraph Build(const OpenDriveMap& map,
                         const LaneGraphOptions& options = {});
  // Rebuilds the graph from the latched message.
  static LaneGraph FromMsg(const nuway_msgs::msg::LaneGraph& msg,
                           const LaneGraphOptions& options = {});
  nuway_msgs::msg::LaneGraph ToMsg() const;

  // Accessors.
  const Lane* lane(std::uint32_t id) const;
  const std::vector<Lane>& lanes() const { return lanes_; }
  const std::vector<TrafficLightMapping>& traffic_lights() const {
    return traffic_lights_;
  }
  const std::vector<StopSign>& stop_signs() const { return stop_signs_; }
  const std::vector<Crosswalk>& crosswalks() const { return crosswalks_; }
  const GeoReference& geo_reference() const { return geo_reference_; }

  std::vector<std::uint32_t> Successors(std::uint32_t id) const;
  std::vector<std::uint32_t> Predecessors(std::uint32_t id) const;
  // Left then right neighbor, zeros dropped.
  std::vector<std::uint32_t> Neighbors(std::uint32_t id) const;
  std::vector<const TrafficLightMapping*> TrafficLightsForLane(
      std::uint32_t id) const;
  std::optional<Eigen::Vector3d> StopLineForLane(std::uint32_t id) const;

  // Lane id for an OpenDRIVE (road, lane) at arc length s along the road
  // (section boundaries are only known for graphs built from a map; a graph
  // rebuilt from the message returns the first matching section).
  std::optional<std::uint32_t> LaneIdAt(int road_id, int lane_id_odr,
                                        double road_s) const;
  // Lane id for an OpenDRIVE (road, lane) whose centerline passes nearest to
  // (x, y); works for graphs rebuilt from the message.
  std::optional<std::uint32_t> LaneIdNear(int road_id, int lane_id_odr,
                                          double x, double y) const;

  // Nearest drivable lane (driving or bidirectional; never a shoulder or a
  // parking lane) by centerline distance with a heading-consistency check.
  std::optional<LaneQuery> NearestLane(double x, double y, double yaw,
                                       double max_dist) const;
  // Every drivable lane whose centerline passes within max_dist, nearest
  // first, with no heading check (for yaw-less route waypoints: either
  // direction).
  std::vector<LaneQuery> LanesNear(double x, double y, double max_dist) const;
  // The per-lane reference line (built lazily on first use, cached).
  const nuway_common::ReferenceLine* reference_line(std::uint32_t id) const;

 private:
  struct Index;
  void Finalize();

  std::vector<Lane> lanes_;
  std::unordered_map<std::uint32_t, std::size_t> index_by_id_;
  std::vector<TrafficLightMapping> traffic_lights_;
  std::vector<StopSign> stop_signs_;
  std::vector<Crosswalk> crosswalks_;
  GeoReference geo_reference_;
  LaneGraphOptions options_;
  std::unique_ptr<Index> index_;
};

}  // namespace nuway_map

#endif  // NUWAY_MAP_LANE_GRAPH_H_
