// RoutePlanner (M0 §2.5): A* over the lane graph through every route waypoint
// in order. No rclcpp.
//
// The lane graph (nuway_map) is a directed graph: a lane's successors are the
// lanes it flows into, its left/right neighbours the same-direction lanes it
// may change into. A route is a lane sequence and its cost is the sum of the
// edge costs along it: a successor edge costs the length of the lane being
// left, a lateral (lane-change) edge costs the flat lane_change_penalty_m, a
// "20 m equivalent" so the search changes lanes only where doing so saves
// more than 20 m of driving. A* (the mechanics are taught in route_planner.cc)
// returns the cheapest sequence under these costs.
//
// The search state is not the lane alone but the pair (lane, waypoints
// reached so far). A waypoint carries no heading, so its goal is the *set* of
// lanes about as near to it as the nearest one (goal_slack_m): at a junction
// several overlapping lanes sit on it, and on a road's centre line both
// directions do. Only some of those lanes lead on towards the next waypoint.
// The original design (M0 §2.5) searched each waypoint-to-waypoint segment
// on its own and concatenated the results, which picked a segment's goal
// lane by that segment's cost alone; at a junction that could be a lane
// whose successors never reach the following waypoint, and the next segment
// then failed or detoured. Searching all waypoints jointly makes the whole
// remaining route pay for the choice, so the goal lane at a junction is the
// one the route actually continues on. A waypoint behind the ego on its
// current lane is reached by looping back to it through the lane's
// successors (docs/milestones/M0_bringup.md, Decisions log, task 8).
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
  // Cost of one lateral edge, in metres of driving it is worth to avoid.
  double lane_change_penalty_m = 20.0;
  // A waypoint farther than this from every lane makes the plan fail.
  double waypoint_max_dist_m = 10.0;
  // An ego pose farther than this from every lane makes the plan fail.
  double ego_max_dist_m = 10.0;
  // Lanes within this much of the nearest lane's offset form the goal set
  // (both directions of a road when the waypoint sits between them).
  double goal_slack_m = 1.0;
  // A waypoint at most this far behind the ego on its current lane counts as
  // already reached (the harness resets the hero onto waypoint 0), instead
  // of being reached by looping back to it.
  double passed_tolerance_m = 5.0;
};

// Result of PlanRoute: the lane sequence and where each waypoint was reached
// on it.
struct RoutePlan {
  std::vector<std::uint32_t> lane_ids;  // ordered, no repeats in a row
  // Index into lane_ids of the lane each waypoint was reached on.
  std::vector<std::size_t> waypoint_lane_index;
  // The ego's arc length along lane_ids.front() when the plan was made (the
  // reference line's first lateral blend starts there, not at the lane's
  // section start).
  double start_s_m = 0.0;
};

// Single-goal A*: from `start` (a lane id) to any lane in `goals`, the
// lanes holding the point `goal_xy` (map frame, metres). Returns the lane
// sequence including both ends, or nullopt when unreachable or `start` is
// unknown. The heuristic is admissible (see RemainingLowerBound in the
// .cc), so the sequence is the cheapest one under the edge costs. PlanRoute
// is the multi-waypoint search the node uses; this one is the building
// block the unit tests exercise.
std::optional<std::vector<std::uint32_t>> ShortestLanePath(
    const nuway_map::LaneGraph& graph, std::uint32_t start,
    const std::vector<std::uint32_t>& goals, const Eigen::Vector2d& goal_xy,
    const RoutePlannerOptions& options);

// Plans through all waypoints in order from the ego pose (map frame, ROS
// convention). `error` receives the reason on failure: no waypoints, ego or
// a waypoint on no lane, a waypoint unreachable, or one lying behind on the
// current lane with no loop back to it.
std::optional<RoutePlan> PlanRoute(const nuway_map::LaneGraph& graph,
                                   const nuway_common::SE2& ego,
                                   const nuway_common::Vector2dList& waypoints,
                                   const RoutePlannerOptions& options,
                                   std::string* error);

}  // namespace nuway_route

#endif  // NUWAY_ROUTE_ROUTE_PLANNER_H_
