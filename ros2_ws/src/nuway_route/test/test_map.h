// Synthetic OpenDRIVE maps shared by the nuway_route tests.
#ifndef NUWAY_ROUTE_TEST_TEST_MAP_H_
#define NUWAY_ROUTE_TEST_TEST_MAP_H_

#include <optional>
#include <string>

#include <gtest/gtest.h>

#include <nuway_map/lane_graph.h>
#include <nuway_map/opendrive_parser.h>

namespace nuway_route {

// Two straight two-lane roads joined by a junction road, like the
// lane_graph_test map: road 1 (x 0..30) -> road 5 (x 30..40) -> road 2
// (x 40..50, two sections). Right lanes -1/-2 run east, lane 1 runs west.
// On road 1 the mark between -1 and -2 allows only -1 -> -2 ("decrease").
constexpr const char* kCorridorXodr = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="4" name="t"/>
  <road name="r1" length="30.0" id="1" junction="-1">
    <link><successor elementType="junction" elementId="9"/></link>
    <type s="0" type="town"><speed max="30" unit="mph"/></type>
    <planView><geometry s="0" x="0" y="0" hdg="0" length="30"><line/></geometry></planView>
    <lanes><laneSection s="0">
      <left><lane id="1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/><roadMark sOffset="0" type="solid" laneChange="none"/></lane></left>
      <center><lane id="0" type="none"><roadMark sOffset="0" type="solid solid" laneChange="none"/></lane></center>
      <right>
        <lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/><roadMark sOffset="0" type="broken" laneChange="decrease"/></lane>
        <lane id="-2" type="driving"><width sOffset="0" a="3.0" b="0" c="0" d="0"/><roadMark sOffset="0" type="solid" laneChange="none"/></lane>
      </right>
    </laneSection></lanes>
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

// A fork: road 1 (x 0..30) enters junction 9, which holds two connecting
// roads that overlap for their first 5 m: road 5 (x 30..40, straight on to
// road 2) and road 6 (x 30..35, on to road 3 which bends right by 0.5 rad).
// Single eastbound lane -1 everywhere.
constexpr const char* kForkXodr = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="4" name="f"/>
  <road name="r1" length="30.0" id="1" junction="-1">
    <link><successor elementType="junction" elementId="9"/></link>
    <planView><geometry s="0" x="0" y="0" hdg="0" length="30"><line/></geometry></planView>
    <lanes><laneSection s="0">
      <center><lane id="0" type="none"/></center>
      <right><lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></right>
    </laneSection></lanes>
  </road>
  <road name="c5" length="10.0" id="5" junction="9">
    <link>
      <predecessor elementType="road" elementId="1" contactPoint="end"/>
      <successor elementType="road" elementId="2" contactPoint="start"/>
    </link>
    <planView><geometry s="0" x="30" y="0" hdg="0" length="10"><line/></geometry></planView>
    <lanes><laneSection s="0">
      <center><lane id="0" type="none"/></center>
      <right><lane id="-1" type="driving"><link><predecessor id="-1"/><successor id="-1"/></link><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></right>
    </laneSection></lanes>
  </road>
  <road name="r2" length="10.0" id="2" junction="-1">
    <link><predecessor elementType="junction" elementId="9"/></link>
    <planView><geometry s="0" x="40" y="0" hdg="0" length="10"><line/></geometry></planView>
    <lanes><laneSection s="0">
      <center><lane id="0" type="none"/></center>
      <right><lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></right>
    </laneSection></lanes>
  </road>
  <road name="c6" length="5.0" id="6" junction="9">
    <link>
      <predecessor elementType="road" elementId="1" contactPoint="end"/>
      <successor elementType="road" elementId="3" contactPoint="start"/>
    </link>
    <planView><geometry s="0" x="30" y="0" hdg="0" length="5"><line/></geometry></planView>
    <lanes><laneSection s="0">
      <center><lane id="0" type="none"/></center>
      <right><lane id="-1" type="driving"><link><predecessor id="-1"/><successor id="-1"/></link><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></right>
    </laneSection></lanes>
  </road>
  <road name="r3" length="15.0" id="3" junction="-1">
    <link><predecessor elementType="junction" elementId="9"/></link>
    <planView><geometry s="0" x="35" y="0" hdg="-0.5" length="15"><line/></geometry></planView>
    <lanes><laneSection s="0">
      <center><lane id="0" type="none"/></center>
      <right><lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></right>
    </laneSection></lanes>
  </road>
  <junction id="9" name="j">
    <connection id="0" incomingRoad="1" connectingRoad="5" contactPoint="start"><laneLink from="-1" to="-1"/></connection>
    <connection id="1" incomingRoad="1" connectingRoad="6" contactPoint="start"><laneLink from="-1" to="-1"/></connection>
  </junction>
</OpenDRIVE>)";

// A closed circle of radius 30 m (one counter-clockwise arc geometry): lane
// -1 lies to the right, i.e. outside at radius 31.75 driving counter-clockwise;
// lane 1 inside at 28.25 driving clockwise.
constexpr const char* kCircleXodr = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="4" name="c"/>
  <road name="ring" length="188.49555921538757" id="7" junction="-1">
    <link>
      <predecessor elementType="road" elementId="7" contactPoint="end"/>
      <successor elementType="road" elementId="7" contactPoint="start"/>
    </link>
    <planView><geometry s="0" x="30" y="0" hdg="1.5707963267948966" length="188.49555921538757"><arc curvature="0.03333333333333333"/></geometry></planView>
    <lanes><laneSection s="0">
      <left><lane id="1" type="driving"><link><predecessor id="1"/><successor id="1"/></link><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></left>
      <center><lane id="0" type="none"/></center>
      <right><lane id="-1" type="driving"><link><predecessor id="-1"/><successor id="-1"/></link><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></right>
    </laneSection></lanes>
  </road>
</OpenDRIVE>)";

inline nuway_map::LaneGraph BuildGraph(const char* xml) {
  std::string error;
  const std::optional<nuway_map::OpenDriveMap> map =
      nuway_map::ParseOpenDrive(xml, &error);
  EXPECT_TRUE(map.has_value()) << error;
  return nuway_map::LaneGraph::Build(map.value_or(nuway_map::OpenDriveMap{}));
}

inline std::uint32_t Id(int road, int section, int lane) {
  return nuway_map::MakeLaneId(road, section, lane);
}

}  // namespace nuway_route

#endif  // NUWAY_ROUTE_TEST_TEST_MAP_H_
