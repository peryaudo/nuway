// Topic, service and parameter name constants of nuway_map (M0).
#ifndef NUWAY_MAP_NAMES_H_
#define NUWAY_MAP_NAMES_H_

namespace nuway_map {

// The one node of this package (docs/milestones/M0_bringup.md §2.4).
constexpr const char* kNodeName = "map_server_node";
// nuway_msgs/srv/NearestLane, for tools and tests (docs/02_interfaces.md
// §3.10); runtime nodes use the LaneGraph library directly. The lane graph
// topic name itself lives in nuway_common (kTopicLaneGraph).
constexpr const char* kServiceNearestLane = "/nuway/map/nearest_lane";

}  // namespace nuway_map

#endif  // NUWAY_MAP_NAMES_H_
