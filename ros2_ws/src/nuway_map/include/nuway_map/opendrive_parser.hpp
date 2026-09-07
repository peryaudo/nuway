// OpenDRIVE 1.4 parser (M0): turns the XML that CARLA's
// world.get_map().to_opendrive() emits into plain structs (roads with
// reference-line geometry, lane offsets, elevation, lane sections, lanes with
// width polynomials and road marks, signals, signal references, objects, and
// junctions) and evaluates the reference line. Everything is in the OpenDRIVE
// inertial frame, which for CARLA's export is already the ROS map frame
// (x east, y north, heading counter-clockwise; docs/02 §1). No exceptions:
// Parse() returns std::nullopt with the error in *error.
#ifndef NUWAY_MAP_OPENDRIVE_PARSER_HPP_
#define NUWAY_MAP_OPENDRIVE_PARSER_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace nuway_map {

// Cubic polynomial a + b ds + c ds^2 + d ds^3 valid from s_offset.
struct Poly3 {
  double s_offset = 0.0;
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
  double d = 0.0;

  double Eval(double s) const {
    const double ds = s - s_offset;
    return a + (b * ds) + (c * ds * ds) + (d * ds * ds * ds);
  }
};

// Evaluates the polynomial whose s_offset is the largest <= s (first if none).
double EvalPiecewise(const std::vector<Poly3>& polys, double s);

enum class GeometryType : std::uint8_t {
  kLine,
  kArc,
  kSpiral,
  kPoly3,
  kParamPoly3
};

// One planView geometry record.
struct OdrGeometry {
  GeometryType type = GeometryType::kLine;
  double s = 0.0;
  double x = 0.0;
  double y = 0.0;
  double hdg = 0.0;
  double length = 0.0;
  double curvature = 0.0;   // arc
  double curv_start = 0.0;  // spiral
  double curv_end = 0.0;    // spiral
  double a = 0.0;           // poly3 (v = a + b u + c u^2 + d u^3)
  double b = 0.0;
  double c = 0.0;
  double d = 0.0;
  double au = 0.0;  // paramPoly3
  double bu = 0.0;
  double cu = 0.0;
  double du = 0.0;
  double av = 0.0;
  double bv = 0.0;
  double cv = 0.0;
  double dv = 0.0;
  bool p_range_normalized = true;
};

// A point on a road reference line.
struct ReferencePoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double hdg = 0.0;
};

// Lane-change permission of a road mark.
enum class LaneChange : std::uint8_t { kNone, kIncrease, kDecrease, kBoth };

struct RoadMark {
  double s_offset = 0.0;
  std::string type;  // broken, solid, none, ...
  LaneChange lane_change = LaneChange::kNone;
};

constexpr int kNoLink = -1000000;

struct OdrLane {
  int id = 0;
  std::string type;
  std::vector<Poly3> widths;
  std::vector<RoadMark> marks;
  int predecessor = kNoLink;  // lane id in the previous section / road
  int successor = kNoLink;
};

struct OdrLaneSection {
  double s = 0.0;
  double s_end = 0.0;          // filled by the parser
  std::vector<OdrLane> left;   // ids +1, +2, ... (ascending)
  std::vector<OdrLane> right;  // ids -1, -2, ... (descending)
  const OdrLane* Find(int lane_id) const;
};

enum class LinkElement : std::uint8_t { kNone, kRoad, kJunction };

struct RoadLink {
  LinkElement element = LinkElement::kNone;
  int id = -1;
  bool contact_start = true;  // contactPoint="start" (roads only)
};

struct OdrValidity {
  int from_lane = 0;
  int to_lane = 0;
};

// <signal>: definition of a signal on this road.
struct OdrSignal {
  int id = -1;
  std::string type;  // "1000001" traffic light, "206" stop sign
  double s = 0.0;
  double t = 0.0;
  double h_offset = 0.0;
  bool orientation_plus = true;
  std::vector<OdrValidity> validity;
};

// <signalReference>: a signal governs lanes of this road at s.
struct OdrSignalReference {
  int id = -1;
  double s = 0.0;
  double t = 0.0;
  bool orientation_plus = true;
  std::vector<OdrValidity> validity;
};

struct OdrObject {
  int id = -1;
  std::string type;  // crosswalk, parking, ...
  double s = 0.0;
  double t = 0.0;
  double hdg = 0.0;  // relative to the road heading at s
  double length = 0.0;
  double width = 0.0;
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
      outline;  // (u, v) corners in the object frame
};

struct OdrRoad {
  int id = -1;
  double length = 0.0;
  int junction = -1;
  RoadLink predecessor;
  RoadLink successor;
  double speed_limit_mps = 0.0;  // 0 = unknown
  std::vector<OdrGeometry> geometry;
  std::vector<Poly3> lane_offsets;
  std::vector<Poly3> elevation;
  std::vector<OdrLaneSection> sections;
  std::vector<OdrSignal> signals;
  std::vector<OdrSignalReference> signal_references;
  std::vector<OdrObject> objects;

  // Reference line at arc length s (clamped to [0, length]).
  ReferencePoint At(double s) const;
  // Point at (s, t): t positive to the left of the reference direction.
  Eigen::Vector3d AtLateral(double s, double t) const;
  // Index of the lane section containing s.
  int SectionIndex(double s) const;
};

struct OdrLaneLink {
  int from = 0;
  int to = 0;
};

struct OdrConnection {
  int id = -1;
  int incoming_road = -1;
  int connecting_road = -1;
  bool contact_start = true;
  std::vector<OdrLaneLink> lane_links;
};

struct OdrJunction {
  int id = -1;
  std::vector<OdrConnection> connections;
};

struct GeoReference {
  bool valid = false;
  double lat0 = 0.0;
  double lon0 = 0.0;
  double alt0 = 0.0;
};

// The parsed map.
struct OpenDriveMap {
  std::map<int, OdrRoad> roads;
  std::map<int, OdrJunction> junctions;
  GeoReference geo_reference;
};

// Parses an OpenDRIVE XML string. On failure returns nullopt and sets *error.
std::optional<OpenDriveMap> ParseOpenDrive(const std::string& xml,
                                           std::string* error);

// Reads and parses a .xodr file.
std::optional<OpenDriveMap> LoadOpenDrive(const std::string& path,
                                          std::string* error);

// Evaluates one geometry record at arc length s (relative to the road start).
ReferencePoint EvalGeometry(const OdrGeometry& geom, double s);

}  // namespace nuway_map

#endif  // NUWAY_MAP_OPENDRIVE_PARSER_HPP_
