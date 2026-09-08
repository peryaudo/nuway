#include "nuway_map/opendrive_parser.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include <pugixml.hpp>

#include <nuway_common/geometry.h>

namespace nuway_map {
namespace {

double Attr(const pugi::xml_node& node, const char* name, double fallback) {
  const pugi::xml_attribute attr = node.attribute(name);
  return attr.empty() ? fallback : attr.as_double(fallback);
}

int AttrInt(const pugi::xml_node& node, const char* name, int fallback) {
  const pugi::xml_attribute attr = node.attribute(name);
  return attr.empty() ? fallback : attr.as_int(fallback);
}

std::string AttrStr(const pugi::xml_node& node, const char* name) {
  return {node.attribute(name).as_string("")};
}

Poly3 ParsePoly3(const pugi::xml_node& node, const char* s_name) {
  Poly3 poly;
  poly.s_offset = Attr(node, s_name, 0.0);
  poly.a = Attr(node, "a", 0.0);
  poly.b = Attr(node, "b", 0.0);
  poly.c = Attr(node, "c", 0.0);
  poly.d = Attr(node, "d", 0.0);
  return poly;
}

LaneChange ParseLaneChange(const std::string& value, const std::string& type) {
  if (value == "both") {
    return LaneChange::kBoth;
  }
  if (value == "increase") {
    return LaneChange::kIncrease;
  }
  if (value == "decrease") {
    return LaneChange::kDecrease;
  }
  if (value == "none") {
    return LaneChange::kNone;
  }
  // No laneChange attribute: derive from the mark type.
  return type == "broken" ? LaneChange::kBoth : LaneChange::kNone;
}

OdrLane ParseLane(const pugi::xml_node& node) {
  OdrLane lane;
  lane.id = AttrInt(node, "id", 0);
  lane.type = AttrStr(node, "type");
  for (const pugi::xml_node& width : node.children("width")) {
    lane.widths.push_back(ParsePoly3(width, "sOffset"));
  }
  for (const pugi::xml_node& mark : node.children("roadMark")) {
    RoadMark road_mark;
    road_mark.s_offset = Attr(mark, "sOffset", 0.0);
    road_mark.type = AttrStr(mark, "type");
    road_mark.lane_change =
        ParseLaneChange(AttrStr(mark, "laneChange"), road_mark.type);
    lane.marks.push_back(road_mark);
  }
  const pugi::xml_node link = node.child("link");
  if (!link.empty()) {
    if (const pugi::xml_node pred = link.child("predecessor")) {
      lane.predecessor = AttrInt(pred, "id", kNoLink);
    }
    if (const pugi::xml_node succ = link.child("successor")) {
      lane.successor = AttrInt(succ, "id", kNoLink);
    }
  }
  return lane;
}

RoadLink ParseRoadLink(const pugi::xml_node& node) {
  RoadLink link;
  if (!node) {
    return link;
  }
  const std::string element = AttrStr(node, "elementType");
  if (element == "road") {
    link.element = LinkElement::kRoad;
  } else if (element == "junction") {
    link.element = LinkElement::kJunction;
  }
  link.id = AttrInt(node, "elementId", -1);
  link.contact_start = AttrStr(node, "contactPoint") != "end";
  return link;
}

std::vector<OdrValidity> ParseValidity(const pugi::xml_node& node) {
  std::vector<OdrValidity> out;
  for (const pugi::xml_node& validity : node.children("validity")) {
    OdrValidity v;
    v.from_lane = AttrInt(validity, "fromLane", 0);
    v.to_lane = AttrInt(validity, "toLane", 0);
    if (v.from_lane > v.to_lane) {
      std::swap(v.from_lane, v.to_lane);
    }
    out.push_back(v);
  }
  return out;
}

double SpeedToMps(double value, const std::string& unit) {
  if (unit == "mph") {
    return value * 0.44704;
  }
  if (unit == "km/h") {
    return value / 3.6;
  }
  return value;
}

OdrGeometry ParseGeometry(const pugi::xml_node& node) {
  OdrGeometry geom;
  geom.s = Attr(node, "s", 0.0);
  geom.x = Attr(node, "x", 0.0);
  geom.y = Attr(node, "y", 0.0);
  geom.hdg = Attr(node, "hdg", 0.0);
  geom.length = Attr(node, "length", 0.0);
  if (const pugi::xml_node arc = node.child("arc")) {
    geom.type = GeometryType::kArc;
    geom.curvature = Attr(arc, "curvature", 0.0);
  } else if (const pugi::xml_node spiral = node.child("spiral")) {
    geom.type = GeometryType::kSpiral;
    geom.curv_start = Attr(spiral, "curvStart", 0.0);
    geom.curv_end = Attr(spiral, "curvEnd", 0.0);
  } else if (const pugi::xml_node poly = node.child("poly3")) {
    geom.type = GeometryType::kPoly3;
    geom.a = Attr(poly, "a", 0.0);
    geom.b = Attr(poly, "b", 0.0);
    geom.c = Attr(poly, "c", 0.0);
    geom.d = Attr(poly, "d", 0.0);
  } else if (const pugi::xml_node ppoly = node.child("paramPoly3")) {
    geom.type = GeometryType::kParamPoly3;
    geom.au = Attr(ppoly, "aU", 0.0);
    geom.bu = Attr(ppoly, "bU", 0.0);
    geom.cu = Attr(ppoly, "cU", 0.0);
    geom.du = Attr(ppoly, "dU", 0.0);
    geom.av = Attr(ppoly, "aV", 0.0);
    geom.bv = Attr(ppoly, "bV", 0.0);
    geom.cv = Attr(ppoly, "cV", 0.0);
    geom.dv = Attr(ppoly, "dV", 0.0);
    geom.p_range_normalized = AttrStr(ppoly, "pRange") != "arcLength";
  } else {
    geom.type = GeometryType::kLine;
  }
  return geom;
}

OdrObject ParseObject(const pugi::xml_node& node) {
  OdrObject object;
  object.id = AttrInt(node, "id", -1);
  object.type = AttrStr(node, "type");
  object.s = Attr(node, "s", 0.0);
  object.t = Attr(node, "t", 0.0);
  object.hdg = Attr(node, "hdg", 0.0);
  object.length = Attr(node, "length", 0.0);
  object.width = Attr(node, "width", 0.0);
  for (const pugi::xml_node& corner :
       node.child("outline").children("cornerLocal")) {
    object.outline.emplace_back(Attr(corner, "u", 0.0), Attr(corner, "v", 0.0));
  }
  return object;
}

OdrRoad ParseRoad(const pugi::xml_node& node) {
  OdrRoad road;
  road.id = AttrInt(node, "id", -1);
  road.length = Attr(node, "length", 0.0);
  road.junction = AttrInt(node, "junction", -1);
  const pugi::xml_node link = node.child("link");
  road.predecessor = ParseRoadLink(link.child("predecessor"));
  road.successor = ParseRoadLink(link.child("successor"));
  for (const pugi::xml_node& type : node.children("type")) {
    if (const pugi::xml_node speed = type.child("speed")) {
      road.speed_limit_mps =
          SpeedToMps(Attr(speed, "max", 0.0), AttrStr(speed, "unit"));
    }
  }
  for (const pugi::xml_node& geom :
       node.child("planView").children("geometry")) {
    road.geometry.push_back(ParseGeometry(geom));
  }
  for (const pugi::xml_node& elev :
       node.child("elevationProfile").children("elevation")) {
    road.elevation.push_back(ParsePoly3(elev, "s"));
  }
  const pugi::xml_node lanes = node.child("lanes");
  for (const pugi::xml_node& offset : lanes.children("laneOffset")) {
    road.lane_offsets.push_back(ParsePoly3(offset, "s"));
  }
  for (const pugi::xml_node& section_node : lanes.children("laneSection")) {
    OdrLaneSection section;
    section.s = Attr(section_node, "s", 0.0);
    for (const pugi::xml_node& lane :
         section_node.child("left").children("lane")) {
      section.left.push_back(ParseLane(lane));
    }
    for (const pugi::xml_node& lane :
         section_node.child("right").children("lane")) {
      section.right.push_back(ParseLane(lane));
    }
    std::sort(section.left.begin(), section.left.end(),
              [](const OdrLane& a, const OdrLane& b) { return a.id < b.id; });
    std::sort(section.right.begin(), section.right.end(),
              [](const OdrLane& a, const OdrLane& b) { return a.id > b.id; });
    road.sections.push_back(section);
  }
  for (std::size_t i = 0; i < road.sections.size(); ++i) {
    road.sections[i].s_end =
        (i + 1 < road.sections.size()) ? road.sections[i + 1].s : road.length;
  }
  const pugi::xml_node signals = node.child("signals");
  for (const pugi::xml_node& sig : signals.children("signal")) {
    OdrSignal signal;
    signal.id = AttrInt(sig, "id", -1);
    signal.type = AttrStr(sig, "type");
    signal.s = Attr(sig, "s", 0.0);
    signal.t = Attr(sig, "t", 0.0);
    signal.h_offset = Attr(sig, "hOffset", 0.0);
    signal.orientation_plus = AttrStr(sig, "orientation") != "-";
    signal.validity = ParseValidity(sig);
    road.signals.push_back(signal);
  }
  for (const pugi::xml_node& ref : signals.children("signalReference")) {
    OdrSignalReference reference;
    reference.id = AttrInt(ref, "id", -1);
    reference.s = Attr(ref, "s", 0.0);
    reference.t = Attr(ref, "t", 0.0);
    reference.orientation_plus = AttrStr(ref, "orientation") != "-";
    reference.validity = ParseValidity(ref);
    road.signal_references.push_back(reference);
  }
  for (const pugi::xml_node& object :
       node.child("objects").children("object")) {
    road.objects.push_back(ParseObject(object));
  }
  return road;
}

OdrJunction ParseJunction(const pugi::xml_node& node) {
  OdrJunction junction;
  junction.id = AttrInt(node, "id", -1);
  for (const pugi::xml_node& conn : node.children("connection")) {
    OdrConnection connection;
    connection.id = AttrInt(conn, "id", -1);
    connection.incoming_road = AttrInt(conn, "incomingRoad", -1);
    connection.connecting_road = AttrInt(conn, "connectingRoad", -1);
    connection.contact_start = AttrStr(conn, "contactPoint") != "end";
    for (const pugi::xml_node& link : conn.children("laneLink")) {
      OdrLaneLink lane_link;
      lane_link.from = AttrInt(link, "from", 0);
      lane_link.to = AttrInt(link, "to", 0);
      connection.lane_links.push_back(lane_link);
    }
    junction.connections.push_back(connection);
  }
  return junction;
}

// Extracts "+key=value" from a proj string.
std::optional<double> ProjValue(const std::string& proj,
                                const std::string& key) {
  const std::string needle = "+" + key + "=";
  const std::size_t pos = proj.find(needle);
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  const std::string rest = proj.substr(pos + needle.size());
  char* end = nullptr;
  const double value = std::strtod(rest.c_str(), &end);
  if (end == rest.c_str()) {
    return std::nullopt;
  }
  return value;
}

GeoReference ParseGeoReference(const pugi::xml_node& header) {
  GeoReference geo;
  const pugi::xml_node node = header.child("geoReference");
  if (!node) {
    return geo;
  }
  const std::string proj = node.text().as_string("");
  const std::optional<double> lat = ProjValue(proj, "lat_0");
  const std::optional<double> lon = ProjValue(proj, "lon_0");
  if (lat.has_value() && lon.has_value()) {
    geo.valid = true;
    geo.lat0 = *lat;
    geo.lon0 = *lon;
  }
  return geo;
}

// Fresnel-type integrals for the clothoid, by Simpson integration over the
// heading polynomial (accurate to ~1e-9 over the lengths CARLA uses).
Eigen::Vector2d SpiralOffset(double curv_start, double curv_rate, double s) {
  constexpr int kSteps = 64;
  const double h = s / kSteps;
  double x = 0.0;
  double y = 0.0;
  const auto theta = [&](double u) {
    return (curv_start * u) + (0.5 * curv_rate * u * u);
  };
  for (int i = 0; i < kSteps; ++i) {
    const double u0 = i * h;
    const double u1 = u0 + (0.5 * h);
    const double u2 = u0 + h;
    x += (h / 6.0) * (std::cos(theta(u0)) + (4.0 * std::cos(theta(u1))) +
                      std::cos(theta(u2)));
    y += (h / 6.0) * (std::sin(theta(u0)) + (4.0 * std::sin(theta(u1))) +
                      std::sin(theta(u2)));
  }
  return Eigen::Vector2d{x, y};
}

}  // namespace

double EvalPiecewise(const std::vector<Poly3>& polys, double s) {
  if (polys.empty()) {
    return 0.0;
  }
  const Poly3* active = &polys.front();
  for (const Poly3& poly : polys) {
    if (poly.s_offset <= s) {
      active = &poly;
    }
  }
  return active->Eval(s);
}

const OdrLane* OdrLaneSection::Find(int lane_id) const {
  for (const OdrLane& lane : left) {
    if (lane.id == lane_id) {
      return &lane;
    }
  }
  for (const OdrLane& lane : right) {
    if (lane.id == lane_id) {
      return &lane;
    }
  }
  return nullptr;
}

ReferencePoint EvalGeometry(const OdrGeometry& geom, double s) {
  const double ds = std::max(0.0, std::min(geom.length, s - geom.s));
  const double cos_h = std::cos(geom.hdg);
  const double sin_h = std::sin(geom.hdg);
  ReferencePoint out;
  double local_x = ds;
  double local_y = 0.0;
  double local_hdg = 0.0;
  switch (geom.type) {
    case GeometryType::kLine:
      break;
    case GeometryType::kArc: {
      const double kappa = geom.curvature;
      if (std::abs(kappa) < 1e-12) {
        break;
      }
      local_hdg = kappa * ds;
      local_x = std::sin(local_hdg) / kappa;
      local_y = (1.0 - std::cos(local_hdg)) / kappa;
      break;
    }
    case GeometryType::kSpiral: {
      const double rate = geom.length > 0.0
                              ? (geom.curv_end - geom.curv_start) / geom.length
                              : 0.0;
      const Eigen::Vector2d offset = SpiralOffset(geom.curv_start, rate, ds);
      local_x = offset.x();
      local_y = offset.y();
      local_hdg = (geom.curv_start * ds) + (0.5 * rate * ds * ds);
      break;
    }
    case GeometryType::kPoly3: {
      // v(u) with u along the initial heading; ds approximates u.
      const double u = ds;
      local_x = u;
      local_y = geom.a + (geom.b * u) + (geom.c * u * u) + (geom.d * u * u * u);
      local_hdg =
          std::atan2(geom.b + (2.0 * geom.c * u) + (3.0 * geom.d * u * u), 1.0);
      break;
    }
    case GeometryType::kParamPoly3: {
      double p = ds;
      if (geom.p_range_normalized) {
        p = geom.length > 0.0 ? ds / geom.length : 0.0;
      }
      local_x =
          geom.au + (geom.bu * p) + (geom.cu * p * p) + (geom.du * p * p * p);
      local_y =
          geom.av + (geom.bv * p) + (geom.cv * p * p) + (geom.dv * p * p * p);
      const double dx = geom.bu + (2.0 * geom.cu * p) + (3.0 * geom.du * p * p);
      const double dy = geom.bv + (2.0 * geom.cv * p) + (3.0 * geom.dv * p * p);
      local_hdg = std::atan2(dy, dx);
      break;
    }
  }
  out.x = geom.x + (cos_h * local_x) - (sin_h * local_y);
  out.y = geom.y + (sin_h * local_x) + (cos_h * local_y);
  out.hdg = nuway_common::WrapAngle(geom.hdg + local_hdg);
  return out;
}

ReferencePoint OdrRoad::At(double s) const {
  const double s_clamped = std::max(0.0, std::min(length, s));
  if (geometry.empty()) {
    return ReferencePoint{};
  }
  const OdrGeometry* active = &geometry.front();
  for (const OdrGeometry& geom : geometry) {
    if (geom.s <= s_clamped + 1e-9) {
      active = &geom;
    }
  }
  ReferencePoint point = EvalGeometry(*active, s_clamped);
  point.z = EvalPiecewise(elevation, s_clamped);
  return point;
}

Eigen::Vector3d OdrRoad::AtLateral(double s, double t) const {
  const ReferencePoint ref = At(s);
  return Eigen::Vector3d{ref.x - (t * std::sin(ref.hdg)),
                         ref.y + (t * std::cos(ref.hdg)), ref.z};
}

int OdrRoad::SectionIndex(double s) const {
  int idx = 0;
  for (std::size_t i = 0; i < sections.size(); ++i) {
    if (sections[i].s <= s + 1e-9) {
      idx = static_cast<int>(i);
    }
  }
  return idx;
}

std::optional<OpenDriveMap> ParseOpenDrive(const std::string& xml,
                                           std::string* error) {
  pugi::xml_document doc;
  const pugi::xml_parse_result result = doc.load_string(xml.c_str());
  if (!result) {
    if (error != nullptr) {
      *error = std::string("XML parse error: ") + result.description();
    }
    return std::nullopt;
  }
  const pugi::xml_node root = doc.child("OpenDRIVE");
  if (!root) {
    if (error != nullptr) {
      *error = "no <OpenDRIVE> root element";
    }
    return std::nullopt;
  }
  OpenDriveMap map;
  map.geo_reference = ParseGeoReference(root.child("header"));
  for (const pugi::xml_node& road : root.children("road")) {
    OdrRoad parsed = ParseRoad(road);
    map.roads[parsed.id] = std::move(parsed);
  }
  for (const pugi::xml_node& junction : root.children("junction")) {
    OdrJunction parsed = ParseJunction(junction);
    map.junctions[parsed.id] = std::move(parsed);
  }
  if (map.roads.empty()) {
    if (error != nullptr) {
      *error = "no roads";
    }
    return std::nullopt;
  }
  return map;
}

std::optional<OpenDriveMap> LoadOpenDrive(const std::string& path,
                                          std::string* error) {
  const std::ifstream file(path);
  if (!file) {
    if (error != nullptr) {
      *error = "cannot open " + path;
    }
    return std::nullopt;
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return ParseOpenDrive(buffer.str(), error);
}

}  // namespace nuway_map
