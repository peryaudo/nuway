// OpenDRIVE 1.4 parser (M0): turns the XML that CARLA's
// world.get_map().to_opendrive() emits into plain structs (roads with
// reference-line geometry, lane offsets, elevation, lane sections, lanes with
// width polynomials and road marks, signals, signal references, objects, and
// junctions) and evaluates the reference line
// (docs/milestones/M0_bringup.md §2.4).
//
// OpenDRIVE in one paragraph. A map is a set of <road>s. Each road has a
// *reference line*: a planar curve parameterised by arc length s in
// [0, length], made of consecutive <planView>/<geometry> records that each
// start at a pose (x, y, hdg) and are a straight line, a circular arc of
// constant curvature, an Euler spiral (clothoid: curvature linear in s) or a
// cubic (poly3 / paramPoly3). Everything else on the road lives in the road's
// (s, t) coordinates: s along the reference line, t perpendicular to it,
// positive to the LEFT of the direction of increasing s. Lanes are stacked
// outward from the reference line in <laneSection>s, each valid from its own
// s up to the next section's s. Lane id 0 is the reference line itself (a
// zero-width separator, never drivable); +1, +2, ... lie to its left and
// -1, -2, ... to its right, in order of distance. A lane's width is a cubic
// polynomial in the distance from the section start (<width sOffset a b c
// d>), and the <laneOffset> polynomial in s shifts the whole stack sideways.
// In right-hand traffic the right lanes (id < 0) are driven towards +s and
// the left lanes (id > 0) towards -s. A <roadMark> sits on the OUTER edge of
// its lane (further from the reference line) and says whether a lane change
// across that edge is allowed. Roads join end to end through <link>
// predecessor/successor records, each naming the other road and the
// contactPoint ("start" or "end") touched there, or feed into a <junction>,
// whose <connection>s enumerate the short connecting roads and the
// lane-to-lane links onto them. <signal>s, <signalReference>s and <object>s
// are placed at (s, t) too.
//
// Everything is in the OpenDRIVE inertial frame, which for CARLA's export is
// already the ROS map frame (x east, y north, heading counter-clockwise;
// docs/02_interfaces.md §1). No exceptions: Parse() returns std::nullopt with
// the error in *error.
#ifndef NUWAY_MAP_OPENDRIVE_PARSER_H_
#define NUWAY_MAP_OPENDRIVE_PARSER_H_

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace nuway_map {

// Cubic polynomial a + b ds + c ds^2 + d ds^3 with ds = s - s_offset, valid
// from s_offset until the next record. OpenDRIVE uses this one shape for lane
// widths (sOffset counted from the lane section start), lane offsets and
// elevation (s counted from the road start); the caller decides which origin
// s is measured from.
struct Poly3 {
  double s_offset = 0.0;
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
  double d = 0.0;

  // Value at s. Does not check that s lies in this record's range; use
  // EvalPiecewise to pick the record first.
  double Eval(double s) const {
    const double ds = s - s_offset;
    return a + (b * ds) + (c * ds * ds) + (d * ds * ds * ds);
  }
};

// Evaluates the polynomial whose s_offset is the largest <= s (first if none):
// OpenDRIVE records are valid from their own offset up to the next record's,
// so this is the piecewise function the list describes. `polys` must be in
// ascending s_offset order, as the exporter writes them. An empty list (a
// road without <laneOffset>, say) evaluates to 0.
double EvalPiecewise(const std::vector<Poly3>& polys, double s);

// Shape of one reference-line segment; see EvalGeometry for the formulas.
enum class GeometryType : std::uint8_t {
  kLine,       // straight, constant heading
  kArc,        // constant curvature (signed: + turns left)
  kSpiral,     // clothoid: curvature linear from curv_start to curv_end
  kPoly3,      // v = a + b u + c u^2 + d u^3 in the record's local frame
  kParamPoly3  // (u(p), v(p)) both cubic in a parameter p
};

// One planView geometry record: the segment of the reference line starting
// at arc length s (from the road start) at pose (x, y, hdg) and running for
// `length` metres. Only the fields of its `type` are meaningful.
struct OdrGeometry {
  GeometryType type = GeometryType::kLine;
  double s = 0.0;  // start arc length along the road, m
  double x = 0.0;  // start point, map frame, m
  double y = 0.0;
  double hdg = 0.0;         // start heading, rad, counter-clockwise from +x
  double length = 0.0;      // arc length of this segment, m
  double curvature = 0.0;   // arc, 1/m
  double curv_start = 0.0;  // spiral, 1/m
  double curv_end = 0.0;    // spiral, 1/m
  double a = 0.0;           // poly3 (v = a + b u + c u^2 + d u^3)
  double b = 0.0;
  double c = 0.0;
  double d = 0.0;
  double au = 0.0;  // paramPoly3: u(p) = au + bu p + cu p^2 + du p^3
  double bu = 0.0;
  double cu = 0.0;
  double du = 0.0;
  double av = 0.0;  // paramPoly3: v(p) = av + bv p + cv p^2 + dv p^3
  double bv = 0.0;
  double cv = 0.0;
  double dv = 0.0;
  // paramPoly3 pRange: true = p runs over [0, 1] along the segment
  // ("normalized"), false = p is the arc length ("arcLength").
  bool p_range_normalized = true;
};

// A point on a road reference line: position in the map frame (m) and the
// tangent heading (rad, counter-clockwise from +x) in the +s direction.
struct ReferencePoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double hdg = 0.0;
};

// Lane-change permission of a road mark, in OpenDRIVE's lane-id terms:
// "increase" allows crossing the mark towards a higher lane id (-2 -> -1,
// +1 -> +2), "decrease" towards a lower one. Which of those is a left or a
// right change depends on the driving direction; lane_graph.cc resolves it.
enum class LaneChange : std::uint8_t { kNone, kIncrease, kDecrease, kBoth };

// One <roadMark> record on the outer edge of a lane, valid from s_offset
// (counted from the lane section start) until the next record.
struct RoadMark {
  double s_offset = 0.0;
  std::string type;  // broken, solid, none, ...
  LaneChange lane_change = LaneChange::kNone;
};

// "No link" sentinel for OdrLane::predecessor / successor (lane ids are
// small signed integers, so any real id is far from this).
constexpr int kNoLink = -1000000;

// One <lane> of a lane section. The links name an OpenDRIVE lane id in the
// neighbouring section of the same road or, at the road's ends, in the linked
// road; they are topological (predecessor = towards -s), not directional, so
// for a left lane (driven towards -s) the predecessor is what comes next.
struct OdrLane {
  int id = 0;
  std::string type;           // driving, shoulder, parking, sidewalk, ...
  std::vector<Poly3> widths;  // sOffset from the section start
  std::vector<RoadMark> marks;
  int predecessor = kNoLink;  // lane id in the previous section / road
  int successor = kNoLink;
};

// One <laneSection>: the lane stack valid for road arc length [s, s_end).
// Lanes are kept sorted by distance from the reference line on each side,
// which is the order their widths accumulate in.
struct OdrLaneSection {
  double s = 0.0;
  double s_end = 0.0;          // filled by the parser
  std::vector<OdrLane> left;   // ids +1, +2, ... (ascending)
  std::vector<OdrLane> right;  // ids -1, -2, ... (descending)
  // The lane with this OpenDRIVE id on either side, or nullptr.
  const OdrLane* Find(int lane_id) const;
};

// What a road <link> points at.
enum class LinkElement : std::uint8_t { kNone, kRoad, kJunction };

// A road's <link>/<predecessor> or <successor>. For a road link,
// contact_start says which end of the other road touches this one: a
// successor with contactPoint "start" continues nose to tail (its s keeps
// counting up), with "end" the other road runs the opposite way. For a
// junction link the junction's connections carry the contact points.
struct RoadLink {
  LinkElement element = LinkElement::kNone;
  int id = -1;
  bool contact_start = true;  // contactPoint="start" (roads only)
};

// <validity fromLane toLane>: the OpenDRIVE lane ids a signal governs,
// normalised so that from_lane <= to_lane.
struct OdrValidity {
  int from_lane = 0;
  int to_lane = 0;
};

// <signal>: definition of a signal on this road. CARLA defines each traffic
// light once (on some junction road, with an empty validity) and attaches it
// to the lanes it governs through <signalReference> records on the junction
// roads (M0 Decisions log, task 7 (f)).
struct OdrSignal {
  int id = -1;
  std::string type;  // "1000001" traffic light, "206" stop sign
  double s = 0.0;
  double t = 0.0;
  double h_offset = 0.0;
  bool orientation_plus = true;
  std::vector<OdrValidity> validity;
};

// <signalReference>: the signal with this id (defined on some road) governs
// the lanes in `validity` of this road at arc length s.
struct OdrSignalReference {
  int id = -1;
  double s = 0.0;
  double t = 0.0;
  bool orientation_plus = true;
  std::vector<OdrValidity> validity;
};

// <object>: a static feature placed at (s, t) on the road, either as an
// explicit outline or as a length x width box, both in an object frame
// rotated by hdg relative to the road heading at s.
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

// One <road>: its reference line (geometry + elevation), the lane stack
// (lane_offsets + sections) and everything placed on it. All per-record
// lists are in ascending s order.
struct OdrRoad {
  int id = -1;
  double length = 0.0;  // total reference-line arc length, m
  int junction = -1;    // id of the junction this road is inside, or -1
  RoadLink predecessor;
  RoadLink successor;
  double speed_limit_mps = 0.0;  // 0 = unknown
  std::vector<OdrGeometry> geometry;
  std::vector<Poly3> lane_offsets;  // s from the road start
  std::vector<Poly3> elevation;     // z(s), s from the road start
  std::vector<OdrLaneSection> sections;
  std::vector<OdrSignal> signals;
  std::vector<OdrSignalReference> signal_references;
  std::vector<OdrObject> objects;

  // Reference line at arc length s (clamped to [0, length]): the geometry
  // record containing s evaluated there, with z from the elevation profile.
  ReferencePoint At(double s) const;
  // Point at road coordinates (s, t): t positive to the left of the +s
  // direction, i.e. the reference point moved t metres along its left normal
  // (-sin hdg, cos hdg). z is the reference line's (no superelevation).
  Eigen::Vector3d AtLateral(double s, double t) const;
  // Index of the lane section containing s (0 when s is before the first).
  int SectionIndex(double s) const;
};

// <laneLink>: lane `from` of the incoming road continues as lane `to` of the
// connecting road.
struct OdrLaneLink {
  int from = 0;
  int to = 0;
};

// <connection>: one way through a junction. Traffic arriving on
// incoming_road enters connecting_road (a short road inside the junction) at
// its start (contact_start) or its end, following the lane links.
struct OdrConnection {
  int id = -1;
  int incoming_road = -1;
  int connecting_road = -1;
  bool contact_start = true;
  std::vector<OdrLaneLink> lane_links;
};

// <junction>: a set of connections; the connecting roads themselves are
// ordinary roads with `junction` set to this id.
struct OdrJunction {
  int id = -1;
  std::vector<OdrConnection> connections;
};

// <geoReference>: the WGS84 origin of the map frame, read from the +lat_0 /
// +lon_0 terms of the proj string (docs/02_interfaces.md §4; used by the M5
// GNSS path). valid is false when the header carries none.
struct GeoReference {
  bool valid = false;
  double lat0 = 0.0;
  double lon0 = 0.0;
  double alt0 = 0.0;
};

// The parsed map, keyed by OpenDRIVE road and junction id.
struct OpenDriveMap {
  std::map<int, OdrRoad> roads;
  std::map<int, OdrJunction> junctions;
  GeoReference geo_reference;
};

// Parses an OpenDRIVE XML string. On failure (malformed XML, no <OpenDRIVE>
// root, no roads) returns nullopt and sets *error when error is non-null.
// Missing attributes take neutral defaults rather than failing.
std::optional<OpenDriveMap> ParseOpenDrive(const std::string& xml,
                                           std::string* error);

// Reads and parses a .xodr file; "cannot open" is reported through *error.
std::optional<OpenDriveMap> LoadOpenDrive(const std::string& path,
                                          std::string* error);

// Evaluates one geometry record at arc length s (relative to the road start;
// clamped to the record's [s, s + length]). Exposed for tests.
ReferencePoint EvalGeometry(const OdrGeometry& geom, double s);

}  // namespace nuway_map

#endif  // NUWAY_MAP_OPENDRIVE_PARSER_H_
