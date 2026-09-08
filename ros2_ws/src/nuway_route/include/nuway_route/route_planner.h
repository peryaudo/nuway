// RoutePlanner (M0 §2.5): A* over the lane graph through every route waypoint
// in order. Node = (lane id, waypoints reached so far); successor edges cost
// the lane's length, lateral neighbor edges cost lane_change_penalty_m. A
// waypoint carries no heading, so its goal is the set of lanes about as near
// to it as the nearest one (goal_slack_m); searching all waypoints jointly
// picks among overlapping junction lanes by where the route goes next. A
// waypoint behind on the current lane is reached by looping back to it. No
// rclcpp.
#ifndef NUWAY_ROUTE_ROUTE_PLANNER_H_
#define NUWAY_ROUTE_ROUTE_PLANNER_H_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <nuway_common/geometry.h>
#include <nuway_map/lane_graph.h>

namespace nuway_route {

struct RoutePlannerOptions {
  double lane_change_penalty_m = 20.0;
  double waypoint_max_dist_m = 10.0;
  double ego_max_dist_m = 10.0;
  // Lanes within this much of the nearest lane's offset form the goal set
  // (both directions of a road when the waypoint sits between them).
  double goal_slack_m = 1.0;
  // A waypoint at most this far behind the ego on its current lane counts as
  // already reached (the harness resets the hero onto waypoint 0), instead
  // of being reached by looping back to it.
  double passed_tolerance_m = 5.0;
};

struct RoutePlan {
  std::vector<std::uint32_t> lane_ids;  // ordered, no repeats in a row
  // Index into lane_ids of the lane each waypoint was reached on.
  std::vector<std::size_t> waypoint_lane_index;
  // The ego's arc length along lane_ids.front() when the plan was made (the
  // reference line's first lateral blend starts there, not at the lane's
  // section start).
  double start_s_m = 0.0;
};

// A* from `start` (a lane id) to any lane in `goals`; returns the lane
// sequence including both ends, or nullopt when unreachable. The heuristic
// is admissible (see RemainingLowerBound in the .cc), so the sequence is the
// cheapest one under the edge costs.
std::optional<std::vector<std::uint32_t>> ShortestLanePath(
    const nuway_map::LaneGraph& graph, std::uint32_t start,
    const std::vector<std::uint32_t>& goals, const Eigen::Vector2d& goal_xy,
    const RoutePlannerOptions& options);

// Plans through all waypoints in order from the ego pose. `error` receives
// the reason on failure.
std::optional<RoutePlan> PlanRoute(const nuway_map::LaneGraph& graph,
                                   const nuway_common::SE2& ego,
                                   const nuway_common::Vector2dList& waypoints,
                                   const RoutePlannerOptions& options,
                                   std::string* error);

}  // namespace nuway_route

#endif  // NUWAY_ROUTE_ROUTE_PLANNER_H_
