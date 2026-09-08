// LaneGraph (M0): lanes with ids per (road, section, lane), topology through
// junctions, neighbors and change flags, speed limits, traffic-light / stop
// sign / crosswalk associations, and the NearestLane query (KD-tree over
// centerline samples + heading check). Built from an OpenDriveMap; converts to
// and from nuway_msgs/LaneGraph so runtime nodes can rebuild it from the
// latched topic (docs/milestones/M0_bringup.md §2.4, docs/02_interfaces.md
// §4). No rclcpp.
//
// The model. A graph node ("lane") is one OpenDRIVE lane of one lane section
// of one road; lane 0 and lane types other than driving, shoulder, parking
// and bidirectional are dropped. Each lane stores its centerline sampled
// every centerline_spacing_m in the DRIVING direction: right lanes
// (OpenDRIVE id < 0) run with the road's s, left lanes (id > 0) against it,
// so a left lane's samples are reversed at build time and its OpenDRIVE
// *predecessor* becomes its graph successor. Edges are successors and
// predecessors in the driving direction: inside a road they follow the lane
// <link>s from section to section; at a road end they follow the road <link>
// either onto a plain neighbouring road or through every <junction>
// connection that lists this lane. The lateral relations are the left and
// right neighbour (the adjacent same-direction drivable lane of the same
// section; never across lane 0, i.e. never into oncoming traffic) and one
// change-allowed flag per side, derived from the <roadMark> records on the
// boundary between the two lanes. Traffic lights, stop signs and crosswalks
// are attached to the lanes they govern or cross.
//
// Ids. A lane id packs (road, section, OpenDRIVE lane id) into a uint32
// (MakeLaneId), so it is the same in every process that builds the graph
// from the same map, is invertible, and is never 0 (0 means "none" in the
// neighbour fields). Signal ids are a hash of (road, signal id) with the
// high bit kept clear: the GT publisher sets that bit on lights it could not
// map to a TrafficLightMapping (M0 §2.2).
//
// Queries. LanesNear finds every drivable lane whose centerline passes
// within max_dist of a point: a KD-tree radius search over all centerline
// samples yields the candidate lanes, each candidate's centerline is treated
// as a reference line and the point is projected onto it (Frenet s along the
// lane, d to its left), and the results are sorted by |d|. NearestLane keeps
// the closest of those whose heading at the foot point is within
// heading_tolerance_rad of the query yaw, so that a car on a two-way road
// matches the lane going its way rather than the oncoming lane at the same
// distance, and a car in a junction (where connecting lanes overlap) matches
// the one it is actually following.
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

// Lane types kept from OpenDRIVE (msg Lane.TYPE_*); values are the wire
// encoding. Only driving and bidirectional are "drivable" for the queries.
enum class LaneType : std::uint8_t {
  kDriving = 0,
  kShoulder = 1,
  kParking = 2,
  kBidirectional = 3,
  kOther = 4,
};

// One graph node. Geometry is in the map frame; all ids are graph lane ids
// (MakeLaneId), and all directions (successor, left, right) are the lane's
// driving direction.
struct Lane {
  std::uint32_t id = 0;
  std::uint32_t road_id = 0;
  int section_idx = 0;  // index into OdrRoad::sections
  int lane_id_odr = 0;  // OpenDRIVE lane id: < 0 right of the reference line
  LaneType type = LaneType::kDriving;
  nuway_common::Vector3dList centerline;    // driving direction, <= 2 m spacing
  std::vector<double> width;                // per point, m
  std::vector<std::uint32_t> successors;    // lanes this one flows into
  std::vector<std::uint32_t> predecessors;  // lanes flowing into this one
  std::uint32_t left_neighbor = 0;          // 0 = none
  std::uint32_t right_neighbor = 0;
  bool left_change_allowed = false;  // road marks permit a change that way
  bool right_change_allowed = false;
  double speed_limit_mps = 0.0;  // 0 = unknown
  double length_m = 0.0;         // polyline length of the centerline
  // Road arc-length range of the lane section; known only for graphs built
  // from a map (FromMsg leaves the range open).
  double section_s_begin = 0.0;
  double section_s_end = 1e300;
};

// A traffic light and the lanes it governs (docs/02_interfaces.md §4). The
// stop line is the first governed lane's centre at the <signalReference> s;
// heading_rad is the direction the light faces, i.e. opposite to the governed
// lane's travel. bulb_position and from_override are filled by the M4
// overrides and stay zero / false here.
struct TrafficLightMapping {
  std::uint32_t id = 0;
  std::vector<std::uint32_t> affected_lane_ids;
  Eigen::Vector3d stop_line = Eigen::Vector3d::Zero();  // map frame, m
  double heading_rad = 0.0;
  Eigen::Vector3d bulb_position = Eigen::Vector3d::Zero();
  bool from_override = false;
};

// A stop sign: the lanes it governs, the stop line on the first of them, and
// a lane-wide box around the line (2 m before to 2 m past) that a planner
// can test the ego footprint against.
struct StopSign {
  std::uint32_t id = 0;
  std::vector<std::uint32_t> affected_lane_ids;
  Eigen::Vector3d stop_line = Eigen::Vector3d::Zero();  // map frame, m
  nuway_common::Vector2dList trigger_volume;            // map frame, m
};

// A crosswalk polygon (map frame) and every lane whose centerline enters it.
struct Crosswalk {
  std::uint32_t id = 0;
  nuway_common::Vector2dList footprint;
  std::vector<std::uint32_t> crossing_lane_ids;
};

// Result of a nearest-lane query: a point expressed in the Frenet frame of
// the lane's centerline.
struct LaneQuery {
  std::uint32_t lane_id = 0;
  double s = 0.0;  // along the lane centerline, m
  double d = 0.0;  // lateral offset, positive left
};

// Build-time knobs (map_server_node parameters, docs/02_interfaces.md §5).
struct LaneGraphOptions {
  double default_speed_limit_mps = 8.33;  // roads without a <speed> record
  double centerline_spacing_m = 1.0;      // upper bound on sample spacing
  // Heading tolerance for NearestLane; lanes whose local heading differs by
  // more than this from the query yaw are skipped.
  double heading_tolerance_rad = 1.3;
};

// Stable lane id for (road, section, OpenDRIVE lane id); never 0. Packs the
// three as road_id * 4096 + section_idx * 64 + (lane_id_odr + 32) + 1, so
// each road owns a block of 4096 ids, each section 64 of them, and lane ids
// in [-32, 31] map to distinct slots (M0 Decisions log, task 7 (b)).
std::uint32_t MakeLaneId(int road_id, int section_idx, int lane_id_odr);
// Stable signal id for (road, signal); never 0 and never with the high bit
// set (TrafficLightMapping.id, StopSign.id, Crosswalk.id).
std::uint32_t MakeSignalId(int road_id, int signal_id);

// The lane-level map. Move-only (it owns a KD-tree); built once per town by
// map_server_node and rebuilt from the message by every consumer.
class LaneGraph {
 public:
  LaneGraph();
  ~LaneGraph();
  LaneGraph(LaneGraph&&) noexcept;
  LaneGraph& operator=(LaneGraph&&) noexcept;
  LaneGraph(const LaneGraph&) = delete;
  LaneGraph& operator=(const LaneGraph&) = delete;

  // Builds the graph from a parsed map: lanes and centerlines, topology,
  // signals, crosswalks, then the query index.
  static LaneGraph Build(const OpenDriveMap& map,
                         const LaneGraphOptions& options = {});
  // Rebuilds the graph from the latched message. Everything ToMsg wrote
  // comes back; section arc-length ranges are not on the wire, so LaneIdAt
  // degrades (see below) and LaneIdNear is the message-safe alternative.
  static LaneGraph FromMsg(const nuway_msgs::msg::LaneGraph& msg,
                           const LaneGraphOptions& options = {});
  // Serialises for /nuway/map/lane_graph (frame_id = map; the caller stamps
  // it).
  nuway_msgs::msg::LaneGraph ToMsg() const;

  // Accessors.
  // The lane with this id, or nullptr.
  const Lane* lane(std::uint32_t id) const;
  const std::vector<Lane>& lanes() const { return lanes_; }
  const std::vector<TrafficLightMapping>& traffic_lights() const {
    return traffic_lights_;
  }
  const std::vector<StopSign>& stop_signs() const { return stop_signs_; }
  const std::vector<Crosswalk>& crosswalks() const { return crosswalks_; }
  const GeoReference& geo_reference() const { return geo_reference_; }

  // Successor / predecessor ids in the driving direction (empty for an
  // unknown id).
  std::vector<std::uint32_t> Successors(std::uint32_t id) const;
  std::vector<std::uint32_t> Predecessors(std::uint32_t id) const;
  // Left then right neighbor, zeros dropped.
  std::vector<std::uint32_t> Neighbors(std::uint32_t id) const;
  // Every traffic light whose affected_lane_ids contain this lane.
  std::vector<const TrafficLightMapping*> TrafficLightsForLane(
      std::uint32_t id) const;
  // The stop line governing this lane: a stop sign's first, else the first
  // traffic light's, else nullopt.
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
  // parking lane) by centerline distance with a heading-consistency check:
  // LanesNear filtered to |heading(s) - yaw| <= heading_tolerance_rad, then
  // the smallest |d|. Inputs in the map frame (m, rad); nullopt when no lane
  // within max_dist passes the check.
  std::optional<LaneQuery> NearestLane(double x, double y, double yaw,
                                       double max_dist) const;
  // Every drivable lane whose centerline passes within max_dist, nearest
  // first, with no heading check (for yaw-less route waypoints: either
  // direction). Points beyond either end of a lane (by more than a small
  // tolerance) do not match that lane.
  std::vector<LaneQuery> LanesNear(double x, double y, double max_dist) const;
  // The per-lane reference line (the centerline as a
  // nuway_common::ReferenceLine, built by Finalize), or nullptr.
  const nuway_common::ReferenceLine* reference_line(std::uint32_t id) const;

 private:
  // KD-tree over the centerline samples plus the per-lane reference lines;
  // defined in lane_graph.cc so nanoflann stays out of this header.
  struct Index;
  // Rebuilds index_by_id_ and index_ from lanes_; the last step of Build and
  // FromMsg.
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
