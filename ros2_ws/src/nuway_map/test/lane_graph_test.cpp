#include "nuway_map/lane_graph.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "nuway_map/opendrive_parser.hpp"

namespace nuway_map {
namespace {

constexpr double kPi = 3.14159265358979323846;

// The minimal two-road map of opendrive_parser_test with a junction added.
constexpr const char* kTestXodr = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="4" name="t">
    <geoReference><![CDATA[+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs]]></geoReference>
  </header>
  <road name="r1" length="30.0" id="1" junction="-1">
    <link><successor elementType="junction" elementId="9"/></link>
    <type s="0" type="town"><speed max="30" unit="mph"/></type>
    <planView><geometry s="0" x="0" y="0" hdg="0" length="30"><line/></geometry></planView>
    <lanes>
      <laneSection s="0">
        <left>
          <lane id="1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/><roadMark sOffset="0" type="solid" laneChange="none"/></lane>
        </left>
        <center><lane id="0" type="none"><roadMark sOffset="0" type="solid solid" laneChange="none"/></lane></center>
        <right>
          <lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/><roadMark sOffset="0" type="broken" laneChange="both"/></lane>
          <lane id="-2" type="driving"><width sOffset="0" a="3.0" b="0" c="0" d="0"/><roadMark sOffset="0" type="solid" laneChange="none"/></lane>
          <lane id="-3" type="sidewalk"><width sOffset="0" a="2.0" b="0" c="0" d="0"/></lane>
        </right>
      </laneSection>
    </lanes>
    <objects>
      <object id="7" type="crosswalk" s="5" t="0" hdg="1.5707963" length="12" width="3" orientation="+">
        <outline><cornerLocal u="-8" v="-1.5"/><cornerLocal u="8" v="-1.5"/><cornerLocal u="8" v="1.5"/><cornerLocal u="-8" v="1.5"/></outline>
      </object>
    </objects>
    <signals>
      <signal id="100" s="28" t="-8" hOffset="0" orientation="+" type="1000001"><validity fromLane="0" toLane="0"/></signal>
      <signalReference id="100" s="28" t="0" orientation="+"><validity fromLane="-2" toLane="-1"/></signalReference>
      <signal id="101" s="2" t="8" hOffset="0" orientation="-" type="206"><validity fromLane="1" toLane="1"/></signal>
    </signals>
  </road>
  <road name="c" length="10.0" id="5" junction="9">
    <link>
      <predecessor elementType="road" elementId="1" contactPoint="end"/>
      <successor elementType="road" elementId="2" contactPoint="start"/>
    </link>
    <planView><geometry s="0" x="30" y="0" hdg="0" length="10"><line/></geometry></planView>
    <lanes><laneSection s="0">
      <left><lane id="1" type="driving"><link><predecessor id="1"/><successor id="1"/></link><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></left>
      <center><lane id="0" type="none"/></center>
      <right>
        <lane id="-1" type="driving"><link><predecessor id="-1"/><successor id="-1"/></link><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane>
        <lane id="-2" type="driving"><link><predecessor id="-2"/><successor id="-2"/></link><width sOffset="0" a="3.0" b="0" c="0" d="0"/></lane>
      </right>
    </laneSection></lanes>
  </road>
  <road name="r2" length="10.0" id="2" junction="-1">
    <link><predecessor elementType="junction" elementId="9"/></link>
    <planView><geometry s="0" x="40" y="0" hdg="0" length="10"><line/></geometry></planView>
    <lanes>
      <laneSection s="0">
        <left><lane id="1" type="driving"><link><successor id="1"/></link><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></left>
        <center><lane id="0" type="none"/></center>
        <right>
          <lane id="-1" type="driving"><link><successor id="-1"/></link><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane>
          <lane id="-2" type="driving"><link><successor id="-2"/></link><width sOffset="0" a="3.0" b="0" c="0" d="0"/></lane>
        </right>
      </laneSection>
      <laneSection s="5">
        <left><lane id="1" type="driving"><link><predecessor id="1"/></link><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></left>
        <center><lane id="0" type="none"/></center>
        <right>
          <lane id="-1" type="driving"><link><predecessor id="-1"/></link><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane>
          <lane id="-2" type="driving"><link><predecessor id="-2"/></link><width sOffset="0" a="3.0" b="0" c="0" d="0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
  <junction id="9" name="j">
    <connection id="0" incomingRoad="1" connectingRoad="5" contactPoint="start">
      <laneLink from="-1" to="-1"/><laneLink from="-2" to="-2"/>
    </connection>
    <connection id="1" incomingRoad="2" connectingRoad="5" contactPoint="end">
      <laneLink from="1" to="1"/>
    </connection>
  </junction>
</OpenDRIVE>)";

// gtest's ASSERT macros are opaque to bugprone-unchecked-optional-access, so
// tests unwrap through these helpers.
OpenDriveMap UnwrapMap(const std::optional<OpenDriveMap>& maybe,
                       const std::string& error) {
  EXPECT_TRUE(maybe.has_value()) << error;
  return maybe.value_or(OpenDriveMap{});
}

LaneQuery UnwrapQuery(const std::optional<LaneQuery>& maybe) {
  EXPECT_TRUE(maybe.has_value());
  return maybe.value_or(LaneQuery{0, -1.0, -1.0});
}

std::uint32_t UnwrapId(const std::optional<std::uint32_t>& maybe) {
  EXPECT_TRUE(maybe.has_value());
  return maybe.value_or(0U);
}

LaneGraph BuildTestGraph() {
  std::string error;
  return LaneGraph::Build(UnwrapMap(ParseOpenDrive(kTestXodr, &error), error));
}

LaneGraph BuildTownGraph(const std::string& xodr) {
  std::string error;
  return LaneGraph::Build(UnwrapMap(LoadOpenDrive(xodr, &error), error));
}

struct FixtureWaypoint {
  int actor_id = 0;
  std::string kind;
  int road_id = 0;
  int section_id = 0;
  int lane_id = 0;
  double s = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  double width = 0.0;
  std::string type;
  int junction = 0;
};

std::vector<std::string> SplitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    out.push_back(field);
  }
  return out;
}

// Reads a fixture CSV; `with_light` selects the traffic-light layout that
// carries the two leading actor_id/kind columns.
std::vector<FixtureWaypoint> ReadFixture(const std::string& path,
                                         bool with_light) {
  std::vector<FixtureWaypoint> out;
  std::ifstream file(path);
  std::string line;
  std::getline(file, line);  // header
  while (std::getline(file, line)) {
    const std::vector<std::string> f = SplitCsv(line);
    std::size_t i = 0;
    FixtureWaypoint wp;
    if (with_light) {
      wp.actor_id = std::stoi(f[i++]);
      wp.kind = f[i++];
    }
    wp.road_id = std::stoi(f[i++]);
    wp.section_id = std::stoi(f[i++]);
    wp.lane_id = std::stoi(f[i++]);
    wp.s = std::stod(f[i++]);
    wp.x = std::stod(f[i++]);
    wp.y = std::stod(f[i++]);
    wp.z = std::stod(f[i++]);
    wp.yaw = std::stod(f[i++]);
    wp.width = std::stod(f[i++]);
    wp.type = f[i++];
    wp.junction = std::stoi(f[i++]);
    out.push_back(wp);
  }
  return out;
}

std::string RepoPath(const std::string& rel) {
  return std::string(NUWAY_REPO_ROOT) + "/" + rel;
}

std::uint32_t Id(int road, int section, int lane) {
  return MakeLaneId(road, section, lane);
}

TEST(LaneGraph, IdsAreStableAndNonZero) {
  EXPECT_NE(MakeLaneId(0, 0, -31), 0U);
  EXPECT_NE(MakeLaneId(1, 0, -1), MakeLaneId(1, 0, 1));
  EXPECT_NE(MakeLaneId(1, 0, -1), MakeLaneId(1, 1, -1));
  EXPECT_NE(MakeLaneId(1, 0, -1), MakeLaneId(2, 0, -1));
  EXPECT_NE(MakeSignalId(1, 100), 0U);
  EXPECT_EQ(MakeSignalId(1, 100), MakeSignalId(1, 100));
  EXPECT_NE(MakeSignalId(1, 100), MakeSignalId(1, 101));
  EXPECT_EQ(MakeSignalId(1, 100) & 0x80000000U, 0U);
}

TEST(LaneGraph, BuildsLanesWithCenterlines) {
  const LaneGraph graph = BuildTestGraph();
  // Road 1: lanes 1, -1, -2 kept (sidewalk dropped); road 5: 3; road 2: 6.
  EXPECT_EQ(graph.lanes().size(), 12U);
  const Lane* right1 = graph.lane(Id(1, 0, -1));
  ASSERT_NE(right1, nullptr);
  EXPECT_EQ(right1->type, LaneType::kDriving);
  EXPECT_EQ(right1->road_id, 1U);
  EXPECT_EQ(right1->lane_id_odr, -1);
  ASSERT_GE(right1->centerline.size(), 31U);
  EXPECT_NEAR(right1->centerline.front().x(), 0.0, 1e-9);
  EXPECT_NEAR(right1->centerline.front().y(), -1.75, 1e-9);
  EXPECT_NEAR(right1->centerline.back().x(), 30.0, 1e-9);
  EXPECT_NEAR(right1->width.front(), 3.5, 1e-9);
  EXPECT_NEAR(right1->length_m, 30.0, 1e-9);
  EXPECT_NEAR(right1->speed_limit_mps, 30.0 * 0.44704, 1e-9);
  const Lane* right2 = graph.lane(Id(1, 0, -2));
  ASSERT_NE(right2, nullptr);
  EXPECT_NEAR(right2->centerline.front().y(), -5.0, 1e-9);
  // Left lanes run against s: the centerline starts at the road end.
  const Lane* left1 = graph.lane(Id(1, 0, 1));
  ASSERT_NE(left1, nullptr);
  EXPECT_NEAR(left1->centerline.front().x(), 30.0, 1e-9);
  EXPECT_NEAR(left1->centerline.front().y(), 1.75, 1e-9);
  EXPECT_NEAR(left1->centerline.back().x(), 0.0, 1e-9);
  // No speed record on road 5: default applies.
  EXPECT_NEAR(graph.lane(Id(5, 0, -1))->speed_limit_mps, 8.33, 1e-9);
  EXPECT_EQ(graph.lane(12345U), nullptr);
}

TEST(LaneGraph, TopologyThroughJunctionsAndSections) {
  const LaneGraph graph = BuildTestGraph();
  // Road 1 right lanes -> connecting road 5 -> road 2 section 0 -> section 1.
  EXPECT_EQ(graph.Successors(Id(1, 0, -1)),
            std::vector<std::uint32_t>{Id(5, 0, -1)});
  EXPECT_EQ(graph.Successors(Id(1, 0, -2)),
            std::vector<std::uint32_t>{Id(5, 0, -2)});
  EXPECT_EQ(graph.Successors(Id(5, 0, -1)),
            std::vector<std::uint32_t>{Id(2, 0, -1)});
  EXPECT_EQ(graph.Successors(Id(2, 0, -1)),
            std::vector<std::uint32_t>{Id(2, 1, -1)});
  EXPECT_TRUE(graph.Successors(Id(2, 1, -1)).empty());
  EXPECT_EQ(graph.Predecessors(Id(5, 0, -1)),
            std::vector<std::uint32_t>{Id(1, 0, -1)});
  EXPECT_EQ(graph.Predecessors(Id(2, 0, -1)),
            std::vector<std::uint32_t>{Id(5, 0, -1)});
  EXPECT_EQ(graph.Predecessors(Id(2, 1, -1)),
            std::vector<std::uint32_t>{Id(2, 0, -1)});
  // Left lanes drive the other way: road 2 section 1 -> section 0 -> 5 -> 1.
  EXPECT_EQ(graph.Successors(Id(2, 1, 1)),
            std::vector<std::uint32_t>{Id(2, 0, 1)});
  EXPECT_EQ(graph.Successors(Id(2, 0, 1)),
            std::vector<std::uint32_t>{Id(5, 0, 1)});
  EXPECT_EQ(graph.Successors(Id(5, 0, 1)),
            std::vector<std::uint32_t>{Id(1, 0, 1)});
  EXPECT_EQ(graph.Predecessors(Id(1, 0, 1)),
            std::vector<std::uint32_t>{Id(5, 0, 1)});
  EXPECT_TRUE(graph.Successors(Id(1, 0, 1)).empty());
}

TEST(LaneGraph, NeighborsAndChangeFlags) {
  const LaneGraph graph = BuildTestGraph();
  const Lane* right1 = graph.lane(Id(1, 0, -1));
  ASSERT_NE(right1, nullptr);
  EXPECT_EQ(right1->left_neighbor, 0U);  // across the centre line: none
  EXPECT_EQ(right1->right_neighbor, Id(1, 0, -2));
  EXPECT_FALSE(right1->left_change_allowed);
  EXPECT_TRUE(right1->right_change_allowed);  // own mark is broken/both
  const Lane* right2 = graph.lane(Id(1, 0, -2));
  ASSERT_NE(right2, nullptr);
  EXPECT_EQ(right2->left_neighbor, Id(1, 0, -1));
  EXPECT_EQ(right2->right_neighbor, 0U);     // sidewalk is not drivable
  EXPECT_TRUE(right2->left_change_allowed);  // crossing -1's broken mark
  EXPECT_FALSE(right2->right_change_allowed);
  const Lane* left1 = graph.lane(Id(1, 0, 1));
  ASSERT_NE(left1, nullptr);
  EXPECT_EQ(left1->left_neighbor, 0U);
  EXPECT_EQ(left1->right_neighbor, 0U);
  EXPECT_EQ(graph.Neighbors(Id(1, 0, -1)),
            std::vector<std::uint32_t>{Id(1, 0, -2)});
  EXPECT_EQ(graph.Neighbors(Id(1, 0, -2)),
            std::vector<std::uint32_t>{Id(1, 0, -1)});
}

TEST(LaneGraph, SignalsAndObjects) {
  const LaneGraph graph = BuildTestGraph();
  ASSERT_EQ(graph.traffic_lights().size(), 1U);
  const TrafficLightMapping& light = graph.traffic_lights().front();
  EXPECT_EQ(light.id, MakeSignalId(1, 100));
  EXPECT_EQ(light.affected_lane_ids,
            (std::vector<std::uint32_t>{Id(1, 0, -2), Id(1, 0, -1)}));
  EXPECT_NEAR(light.stop_line.x(), 28.0, 1e-9);
  EXPECT_NEAR(light.stop_line.y(), -5.0, 1e-9);  // first affected lane: -2
  EXPECT_NEAR(std::abs(light.heading_rad), kPi, 1e-9);  // faces +s traffic
  EXPECT_FALSE(light.from_override);
  EXPECT_EQ(graph.TrafficLightsForLane(Id(1, 0, -1)).size(), 1U);
  EXPECT_TRUE(graph.TrafficLightsForLane(Id(1, 0, 1)).empty());

  ASSERT_EQ(graph.stop_signs().size(), 1U);
  const StopSign& sign = graph.stop_signs().front();
  EXPECT_EQ(sign.affected_lane_ids, std::vector<std::uint32_t>{Id(1, 0, 1)});
  EXPECT_NEAR(sign.stop_line.x(), 2.0, 1e-9);
  EXPECT_NEAR(sign.stop_line.y(), 1.75, 1e-9);
  ASSERT_EQ(sign.trigger_volume.size(), 4U);
  const Eigen::Vector3d stop =
      graph.StopLineForLane(Id(1, 0, 1)).value_or(Eigen::Vector3d::Zero());
  EXPECT_NEAR(stop.x(), 2.0, 1e-9);
  EXPECT_FALSE(graph.StopLineForLane(Id(2, 0, 1)).has_value());
  // A lane governed by a light but no sign reports the light's stop line.
  EXPECT_NEAR(
      graph.StopLineForLane(Id(1, 0, -1)).value_or(Eigen::Vector3d::Zero()).x(),
      28.0, 1e-9);

  ASSERT_EQ(graph.crosswalks().size(), 1U);
  const Crosswalk& crosswalk = graph.crosswalks().front();
  EXPECT_EQ(crosswalk.footprint.size(), 4U);
  // u along the crosswalk (rotated 90 deg from the road): spans y in [-8, 8]
  // and x in [3.5, 6.5]; lanes 1, -1, -2 of road 1 cross it.
  EXPECT_EQ(crosswalk.crossing_lane_ids.size(), 3U);
  EXPECT_TRUE(graph.geo_reference().valid);
}

TEST(LaneGraph, NearestLaneUsesHeading) {
  const LaneGraph graph = BuildTestGraph();
  // Between lanes -1 (y=-1.75) and -2 (y=-5), closer to -1, heading east.
  LaneQuery q = UnwrapQuery(graph.NearestLane(12.0, -2.5, 0.0, 3.0));
  EXPECT_EQ(q.lane_id, Id(1, 0, -1));
  EXPECT_NEAR(q.s, 12.0, 1e-6);
  EXPECT_NEAR(q.d, -0.75, 1e-6);
  // Same point heading west: only the left lane (y=+1.75) matches.
  q = UnwrapQuery(graph.NearestLane(12.0, -2.5, kPi, 5.0));
  EXPECT_EQ(q.lane_id, Id(1, 0, 1));
  EXPECT_NEAR(q.s, 18.0, 1e-6);
  EXPECT_NEAR(q.d, 4.25, 1e-6);
  // Without a heading, both directions qualify, nearest first.
  const std::vector<LaneQuery> near = graph.LanesNear(12.0, -2.5, 5.0);
  ASSERT_EQ(near.size(), 3U);
  EXPECT_EQ(near[0].lane_id, Id(1, 0, -1));
  EXPECT_EQ(near[1].lane_id, Id(1, 0, -2));
  EXPECT_EQ(near[2].lane_id, Id(1, 0, 1));
  // Out of range.
  EXPECT_FALSE(graph.NearestLane(12.0, -2.5, kPi, 1.0).has_value());
  EXPECT_FALSE(graph.NearestLane(500.0, 500.0, 0.0, 10.0).has_value());
  EXPECT_EQ(UnwrapId(graph.LaneIdAt(2, -1, 7.0)), Id(2, 1, -1));
  EXPECT_EQ(UnwrapId(graph.LaneIdAt(2, -1, 2.0)), Id(2, 0, -1));
  EXPECT_FALSE(graph.LaneIdAt(77, -1, 2.0).has_value());
  EXPECT_EQ(UnwrapId(graph.LaneIdNear(2, -1, 47.0, -1.0)), Id(2, 1, -1));
}

TEST(LaneGraph, MessageRoundTrip) {
  const LaneGraph graph = BuildTestGraph();
  const nuway_msgs::msg::LaneGraph msg = graph.ToMsg();
  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_EQ(msg.lanes.size(), graph.lanes().size());
  EXPECT_TRUE(msg.has_georeference);
  const LaneGraph back = LaneGraph::FromMsg(msg);
  ASSERT_EQ(back.lanes().size(), graph.lanes().size());
  for (const Lane& lane : graph.lanes()) {
    const Lane* other = back.lane(lane.id);
    ASSERT_NE(other, nullptr);
    EXPECT_EQ(other->road_id, lane.road_id);
    EXPECT_EQ(other->section_idx, lane.section_idx);
    EXPECT_EQ(other->lane_id_odr, lane.lane_id_odr);
    EXPECT_EQ(other->type, lane.type);
    EXPECT_EQ(other->successors, lane.successors);
    EXPECT_EQ(other->predecessors, lane.predecessors);
    EXPECT_EQ(other->left_neighbor, lane.left_neighbor);
    EXPECT_EQ(other->right_change_allowed, lane.right_change_allowed);
    ASSERT_EQ(other->centerline.size(), lane.centerline.size());
    EXPECT_NEAR((other->centerline.back() - lane.centerline.back()).norm(), 0.0,
                1e-9);
    EXPECT_NEAR(other->speed_limit_mps, lane.speed_limit_mps, 1e-5);
  }
  EXPECT_EQ(back.traffic_lights().size(), 1U);
  EXPECT_EQ(back.stop_signs().front().trigger_volume.size(), 4U);
  EXPECT_EQ(back.crosswalks().front().crossing_lane_ids.size(), 3U);
  const LaneQuery q = UnwrapQuery(back.NearestLane(12.0, -2.5, 0.0, 3.0));
  EXPECT_EQ(q.lane_id, Id(1, 0, -1));
}

// Parser centerlines against CARLA's own waypoints (M0 §2.4): every fixture
// waypoint must lie within 0.15 m of our centerline for the same
// (road, section, lane), with a consistent heading.
class CarlaTownFixture : public ::testing::TestWithParam<const char*> {};

TEST_P(CarlaTownFixture, CenterlinesMatchCarlaWaypoints) {
  const std::string town = GetParam();
  const std::string xodr = RepoPath("data/maps/" + town + "/map.xodr");
  if (!std::filesystem::exists(xodr)) {
    GTEST_SKIP() << xodr << " absent (export it with world_manager)";
  }
  std::string lower = town;
  for (char& c : lower) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  const LaneGraph graph = BuildTownGraph(xodr);
  const std::vector<FixtureWaypoint> waypoints = ReadFixture(
      RepoPath("tests/fixtures/" + lower + "_waypoints.csv"), false);
  ASSERT_GT(waypoints.size(), 1000U);
  int missing = 0;
  int far = 0;
  double worst = 0.0;
  for (const FixtureWaypoint& wp : waypoints) {
    const Lane* lane = graph.lane(Id(wp.road_id, wp.section_id, wp.lane_id));
    if (lane == nullptr) {
      ++missing;
      continue;
    }
    const nuway_common::ReferenceLine* line = graph.reference_line(lane->id);
    const nuway_common::FrenetPoint frenet =
        line->ToFrenet(wp.x, wp.y, 2.0)
            .value_or(nuway_common::FrenetPoint{0.0, 2.0});
    const double dist = std::abs(frenet.d);
    worst = std::max(worst, dist);
    if (dist > 0.15) {
      ++far;
      if (far <= 5) {
        ADD_FAILURE() << town << " road " << wp.road_id << " lane "
                      << wp.lane_id << " s " << wp.s << ": " << dist
                      << " m off";
      }
      continue;
    }
    const double heading_err =
        std::abs(nuway_common::WrapAngle(line->HeadingAt(frenet.s) - wp.yaw));
    EXPECT_LT(heading_err, 0.1) << town << " road " << wp.road_id << " lane "
                                << wp.lane_id << " s " << wp.s;
    EXPECT_NEAR(lane->width[0], wp.width, 0.5);
  }
  EXPECT_EQ(missing, 0);
  EXPECT_EQ(far, 0);
  RecordProperty("worst_offset_m", std::to_string(worst));
}

TEST_P(CarlaTownFixture, TrafficLightsMatchCarlaAffectedWaypoints) {
  const std::string town = GetParam();
  const std::string xodr = RepoPath("data/maps/" + town + "/map.xodr");
  if (!std::filesystem::exists(xodr)) {
    GTEST_SKIP() << xodr << " absent (export it with world_manager)";
  }
  std::string lower = town;
  for (char& c : lower) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  const LaneGraph graph = BuildTownGraph(xodr);
  const std::vector<FixtureWaypoint> rows = ReadFixture(
      RepoPath("tests/fixtures/" + lower + "_traffic_lights.csv"), true);
  std::map<int, std::vector<FixtureWaypoint>> affected;
  std::map<int, std::vector<FixtureWaypoint>> stops;
  for (const FixtureWaypoint& wp : rows) {
    (wp.kind == "affected" ? affected : stops)[wp.actor_id].push_back(wp);
  }
  ASSERT_GT(affected.size(), 10U);
  // CARLA derives its affected waypoints from the same <signalReference>
  // validity records, so every CARLA light must map to exactly one of our
  // mappings through the lanes it governs (the GT traffic-light publisher
  // matches on this, M0 §2.2), and that mapping's stop line sits on one of
  // those lanes at the reference s, i.e. within 1 m of an affected waypoint.
  int unmatched = 0;
  int ambiguous = 0;
  int far = 0;
  std::map<std::uint32_t, int> actors_per_mapping;
  for (const auto& [actor_id, wps] : affected) {
    std::set<std::uint32_t> mappings;
    for (const FixtureWaypoint& wp : wps) {
      const std::uint32_t lane_id =
          UnwrapId(graph.LaneIdNear(wp.road_id, wp.lane_id, wp.x, wp.y));
      for (const TrafficLightMapping* light :
           graph.TrafficLightsForLane(lane_id)) {
        mappings.insert(light->id);
      }
    }
    if (mappings.empty()) {
      ++unmatched;
      if (unmatched <= 5) {
        ADD_FAILURE() << town << " light actor " << actor_id
                      << " unmatched at road " << wps.front().road_id
                      << " lane " << wps.front().lane_id;
      }
      continue;
    }
    if (mappings.size() > 1) {
      ++ambiguous;
    }
    for (const std::uint32_t id : mappings) {
      ++actors_per_mapping[id];
    }
    double nearest = 1e9;
    for (const TrafficLightMapping& light : graph.traffic_lights()) {
      if (mappings.count(light.id) == 0U) {
        continue;
      }
      for (const FixtureWaypoint& wp : wps) {
        nearest = std::min(nearest, std::hypot(light.stop_line.x() - wp.x,
                                               light.stop_line.y() - wp.y));
      }
    }
    if (nearest > 1.0) {
      ++far;
      if (far <= 5) {
        ADD_FAILURE() << town << " light actor " << actor_id << " stop line "
                      << nearest << " m from its affected waypoints";
      }
    }
  }
  EXPECT_EQ(unmatched, 0);
  EXPECT_EQ(ambiguous, 0);
  EXPECT_EQ(far, 0);
  for (const auto& [id, count] : actors_per_mapping) {
    EXPECT_EQ(count, 1) << "mapping " << id << " matched " << count
                        << " actors";
  }
  EXPECT_EQ(graph.traffic_lights().size(), affected.size());
  // Each mapping id is unique and never carries the "unmapped" high bit.
  std::set<std::uint32_t> ids;
  for (const TrafficLightMapping& light : graph.traffic_lights()) {
    EXPECT_TRUE(ids.insert(light.id).second);
    EXPECT_EQ(light.id & 0x80000000U, 0U);
    EXPECT_FALSE(light.affected_lane_ids.empty()) << "light " << light.id;
  }
  // CARLA's stop waypoints sit at its trigger-volume edge on the incoming
  // road, 1.5-8 m upstream of the junction entry (decisions log, task 7):
  // they are a different notion and only a loose sanity bound applies.
  for (const auto& [actor_id, wps] : stops) {
    double nearest = 1e9;
    for (const TrafficLightMapping& light : graph.traffic_lights()) {
      for (const FixtureWaypoint& wp : wps) {
        nearest = std::min(nearest, std::hypot(light.stop_line.x() - wp.x,
                                               light.stop_line.y() - wp.y));
      }
    }
    EXPECT_LT(nearest, 15.0) << town << " light actor " << actor_id;
  }
}

INSTANTIATE_TEST_SUITE_P(Towns, CarlaTownFixture,
                         ::testing::Values("Town03", "Town05"));

}  // namespace
}  // namespace nuway_map
