#include "nuway_map/opendrive_parser.h"

#include <cmath>
#include <filesystem>
#include <string>

#include <gtest/gtest.h>

namespace nuway_map {
namespace {

constexpr double kPi = 3.14159265358979323846;

constexpr const char* kMinimalXodr = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="4" name="t">
    <geoReference><![CDATA[+proj=tmerc +lat_0=48.5 +lon_0=9.25 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs]]></geoReference>
  </header>
  <road name="r1" length="30.0" id="1" junction="-1">
    <link>
      <successor elementType="road" elementId="2" contactPoint="start"/>
    </link>
    <type s="0" type="town"><speed max="30" unit="mph"/></type>
    <planView>
      <geometry s="0" x="0" y="0" hdg="0" length="10"><line/></geometry>
      <geometry s="10" x="10" y="0" hdg="0" length="20"><arc curvature="0.05"/></geometry>
    </planView>
    <elevationProfile><elevation s="0" a="1" b="0.1" c="0" d="0"/></elevationProfile>
    <lanes>
      <laneOffset s="0" a="0" b="0" c="0" d="0"/>
      <laneSection s="0">
        <left>
          <lane id="1" type="driving" level="false">
            <link><successor id="1"/></link>
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
            <roadMark sOffset="0" type="solid" laneChange="none"/>
          </lane>
        </left>
        <center><lane id="0" type="none" level="false"><roadMark sOffset="0" type="solid solid" laneChange="none"/></lane></center>
        <right>
          <lane id="-1" type="driving" level="false">
            <link><successor id="-1"/></link>
            <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
            <roadMark sOffset="0" type="broken" laneChange="both"/>
          </lane>
          <lane id="-2" type="driving" level="false">
            <link><successor id="-2"/></link>
            <width sOffset="0" a="3.0" b="0" c="0" d="0"/>
            <roadMark sOffset="0" type="solid" laneChange="none"/>
          </lane>
          <lane id="-3" type="sidewalk" level="false">
            <width sOffset="0" a="2.0" b="0" c="0" d="0"/>
          </lane>
        </right>
      </laneSection>
    </lanes>
    <objects>
      <object id="7" type="crosswalk" s="5" t="0" hdg="1.5707963" length="12" width="3" orientation="+">
        <outline>
          <cornerLocal u="-6" v="-1.5"/><cornerLocal u="6" v="-1.5"/>
          <cornerLocal u="6" v="1.5"/><cornerLocal u="-6" v="1.5"/>
        </outline>
      </object>
    </objects>
    <signals>
      <signal id="100" s="28" t="-8" hOffset="0" orientation="+" type="1000001">
        <validity fromLane="0" toLane="0"/>
      </signal>
      <signalReference id="100" s="28" t="0" orientation="+">
        <validity fromLane="-2" toLane="-1"/>
      </signalReference>
      <signal id="101" s="2" t="8" hOffset="0" orientation="-" type="206">
        <validity fromLane="1" toLane="1"/>
      </signal>
    </signals>
  </road>
  <road name="r2" length="10.0" id="2" junction="-1">
    <link><predecessor elementType="road" elementId="1" contactPoint="end"/></link>
    <planView>
      <geometry s="0" x="0" y="0" hdg="0" length="10"><line/></geometry>
    </planView>
    <lanes>
      <laneSection s="0">
        <left><lane id="1" type="driving"><link><predecessor id="1"/></link><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></left>
        <center><lane id="0" type="none"/></center>
        <right>
          <lane id="-1" type="driving"><link><predecessor id="-1"/></link><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane>
          <lane id="-2" type="driving"><link><predecessor id="-2"/></link><width sOffset="0" a="3.0" b="0" c="0" d="0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

// gtest's ASSERT macros are opaque to bugprone-unchecked-optional-access, so
// tests unwrap through this helper.
OpenDriveMap Unwrap(const std::optional<OpenDriveMap>& maybe,
                    const std::string& error) {
  EXPECT_TRUE(maybe.has_value()) << error;
  return maybe.value_or(OpenDriveMap{});
}

OpenDriveMap ParseTestMap(const char* xml) {
  std::string error;
  return Unwrap(ParseOpenDrive(xml, &error), error);
}

TEST(OpenDriveParser, ParsesMinimalMap) {
  const OpenDriveMap map = ParseTestMap(kMinimalXodr);
  ASSERT_EQ(map.roads.size(), 2U);
  const OdrRoad& road = map.roads.at(1);
  EXPECT_DOUBLE_EQ(road.length, 30.0);
  EXPECT_EQ(road.successor.element, LinkElement::kRoad);
  EXPECT_EQ(road.successor.id, 2);
  EXPECT_NEAR(road.speed_limit_mps, 30.0 * 0.44704, 1e-9);
  ASSERT_EQ(road.geometry.size(), 2U);
  EXPECT_EQ(road.geometry[1].type, GeometryType::kArc);
  ASSERT_EQ(road.sections.size(), 1U);
  EXPECT_DOUBLE_EQ(road.sections[0].s_end, 30.0);
  EXPECT_EQ(road.sections[0].left.size(), 1U);
  EXPECT_EQ(road.sections[0].right.size(), 3U);
  EXPECT_EQ(road.sections[0].right[0].id, -1);
  EXPECT_EQ(road.sections[0].right[0].marks[0].lane_change, LaneChange::kBoth);
  EXPECT_EQ(road.sections[0].right[1].marks[0].lane_change, LaneChange::kNone);
  EXPECT_EQ(road.sections[0].right[0].successor, -1);
  ASSERT_EQ(road.signals.size(), 2U);
  EXPECT_EQ(road.signals[0].type, "1000001");
  ASSERT_EQ(road.signal_references.size(), 1U);
  EXPECT_EQ(road.signal_references[0].validity[0].from_lane, -2);
  EXPECT_EQ(road.signal_references[0].validity[0].to_lane, -1);
  ASSERT_EQ(road.objects.size(), 1U);
  EXPECT_EQ(road.objects[0].outline.size(), 4U);
  EXPECT_TRUE(map.geo_reference.valid);
  EXPECT_DOUBLE_EQ(map.geo_reference.lat0, 48.5);
  EXPECT_DOUBLE_EQ(map.geo_reference.lon0, 9.25);
}

TEST(OpenDriveParser, RejectsBadInput) {
  std::string error;
  EXPECT_FALSE(ParseOpenDrive("<not xml", &error).has_value());
  EXPECT_FALSE(error.empty());
  EXPECT_FALSE(ParseOpenDrive("<OpenDRIVE/>", &error).has_value());
  EXPECT_FALSE(LoadOpenDrive("/nonexistent/map.xodr", &error).has_value());
}

TEST(OpenDriveParser, EvaluatesLineAndArc) {
  const OpenDriveMap map = ParseTestMap(kMinimalXodr);
  const OdrRoad& road = map.roads.at(1);
  const ReferencePoint p5 = road.At(5.0);
  EXPECT_NEAR(p5.x, 5.0, 1e-12);
  EXPECT_NEAR(p5.y, 0.0, 1e-12);
  EXPECT_NEAR(p5.hdg, 0.0, 1e-12);
  EXPECT_NEAR(p5.z, 1.5, 1e-12);
  // Arc of curvature 0.05 (radius 20) starting at (10, 0): after its full
  // 20 m the heading is 1 rad and the point is at (10 + 20 sin 1, 20 (1 - cos
  // 1)).
  const ReferencePoint pq = road.At(30.0);
  EXPECT_NEAR(pq.x, 10.0 + (20.0 * std::sin(1.0)), 1e-9);
  EXPECT_NEAR(pq.y, 20.0 * (1.0 - std::cos(1.0)), 1e-9);
  EXPECT_NEAR(pq.hdg, 1.0, 1e-9);
  // Beyond the road length the evaluation clamps.
  EXPECT_NEAR(road.At(45.0).hdg, 1.0, 1e-9);
  // Lateral offset: +t is to the left of the heading.
  const Eigen::Vector3d left = road.AtLateral(5.0, 2.0);
  EXPECT_NEAR(left.y(), 2.0, 1e-12);
  const Eigen::Vector3d right = road.AtLateral(5.0, -2.0);
  EXPECT_NEAR(right.y(), -2.0, 1e-12);
}

TEST(OpenDriveParser, SpiralMatchesArcLimits) {
  // A spiral with constant curvature is an arc.
  OdrGeometry spiral;
  spiral.type = GeometryType::kSpiral;
  spiral.length = 20.0;
  spiral.curv_start = 0.05;
  spiral.curv_end = 0.05;
  OdrGeometry arc;
  arc.type = GeometryType::kArc;
  arc.length = 20.0;
  arc.curvature = 0.05;
  for (const double s : {0.0, 5.0, 12.5, 20.0}) {
    const ReferencePoint a = EvalGeometry(spiral, s);
    const ReferencePoint b = EvalGeometry(arc, s);
    EXPECT_NEAR(a.x, b.x, 1e-7);
    EXPECT_NEAR(a.y, b.y, 1e-7);
    EXPECT_NEAR(a.hdg, b.hdg, 1e-9);
  }
  // Curvature ramps from 0 to 0.1 over 20 m: heading at the end is the
  // integral 0.5 * 0.1 * 20 = 1 rad, and the tangent condition holds along it.
  spiral.curv_start = 0.0;
  spiral.curv_end = 0.1;
  const ReferencePoint end = EvalGeometry(spiral, 20.0);
  EXPECT_NEAR(end.hdg, 1.0, 1e-9);
  const ReferencePoint mid = EvalGeometry(spiral, 10.0);
  const ReferencePoint mid2 = EvalGeometry(spiral, 10.001);
  EXPECT_NEAR(std::atan2(mid2.y - mid.y, mid2.x - mid.x), mid.hdg, 1e-3);
}

TEST(OpenDriveParser, ParamPoly3AndPoly3) {
  OdrGeometry pp;
  pp.type = GeometryType::kParamPoly3;
  pp.length = 10.0;
  pp.bu = 10.0;  // u = 10 p, v = 2 p^2 -> ends at (10, 2)
  pp.cv = 2.0;
  const ReferencePoint end = EvalGeometry(pp, 10.0);
  EXPECT_NEAR(end.x, 10.0, 1e-12);
  EXPECT_NEAR(end.y, 2.0, 1e-12);
  EXPECT_NEAR(end.hdg, std::atan2(4.0, 10.0), 1e-12);
  OdrGeometry p3;
  p3.type = GeometryType::kPoly3;
  p3.length = 10.0;
  p3.hdg = kPi / 2.0;
  p3.b = 0.5;
  const ReferencePoint tip = EvalGeometry(p3, 4.0);
  EXPECT_NEAR(tip.x, -2.0, 1e-12);
  EXPECT_NEAR(tip.y, 4.0, 1e-12);
}

TEST(OpenDriveParser, PiecewisePolynomials) {
  const std::vector<Poly3> polys = {{0.0, 1.0, 0.0, 0.0, 0.0},
                                    {10.0, 2.0, 1.0, 0.0, 0.0}};
  EXPECT_DOUBLE_EQ(EvalPiecewise(polys, 5.0), 1.0);
  EXPECT_DOUBLE_EQ(EvalPiecewise(polys, 12.0), 4.0);
  EXPECT_DOUBLE_EQ(EvalPiecewise({}, 3.0), 0.0);
}

TEST(OpenDriveParser, LoadsCarlaTowns) {
  for (const char* town : {"Town03", "Town05"}) {
    const std::string path =
        std::string(NUWAY_REPO_ROOT) + "/data/maps/" + town + "/map.xodr";
    if (!std::filesystem::exists(path)) {
      GTEST_SKIP() << path << " absent (export it with world_manager)";
    }
    std::string error;
    const OpenDriveMap map = Unwrap(LoadOpenDrive(path, &error), error);
    EXPECT_GT(map.roads.size(), 100U);
    EXPECT_GT(map.junctions.size(), 5U);
    EXPECT_TRUE(map.geo_reference.valid);
    int lights = 0;
    for (const auto& [id, road] : map.roads) {
      for (const OdrSignal& signal : road.signals) {
        lights += signal.type == "1000001" ? 1 : 0;
      }
    }
    EXPECT_GT(lights, 10);
  }
}

}  // namespace
}  // namespace nuway_map
