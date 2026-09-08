#include "nuway_route/route_planner.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "test_map.h"

namespace nuway_route {
namespace {

using Ids = std::vector<std::uint32_t>;

// gtest's ASSERT macros are opaque to bugprone-unchecked-optional-access, so
// tests unwrap through these helpers.
RoutePlan Unwrap(const std::optional<RoutePlan>& maybe,
                 const std::string& error) {
  EXPECT_TRUE(maybe.has_value()) << error;
  return maybe.value_or(RoutePlan{});
}

Ids UnwrapIds(const std::optional<Ids>& maybe) {
  EXPECT_TRUE(maybe.has_value());
  return maybe.value_or(Ids{});
}

TEST(RoutePlanner, ShortestPathFollowsSuccessors) {
  const nuway_map::LaneGraph graph = BuildGraph(kCorridorXodr);
  const RoutePlannerOptions options;
  const Ids path =
      UnwrapIds(ShortestLanePath(graph, Id(1, 0, -1), {Id(2, 1, -1)},
                                 Eigen::Vector2d{48.0, -1.75}, options));
  EXPECT_EQ(path,
            (Ids{Id(1, 0, -1), Id(5, 0, -1), Id(2, 0, -1), Id(2, 1, -1)}));
  // Unreachable: the westbound lane from an eastbound start.
  EXPECT_FALSE(ShortestLanePath(graph, Id(1, 0, -1), {Id(1, 0, 1)},
                                Eigen::Vector2d{0.0, 1.75}, options)
                   .has_value());
  EXPECT_FALSE(ShortestLanePath(graph, 12345U, {Id(1, 0, 1)},
                                Eigen::Vector2d{0.0, 1.75}, options)
                   .has_value());
}

TEST(RoutePlanner, LaneChangeCostsThePenalty) {
  const nuway_map::LaneGraph graph = BuildGraph(kCorridorXodr);
  RoutePlannerOptions options;
  // Goal on lane -2 of road 2: change from -1 to -2 on road 1 (allowed by
  // -1's "decrease" mark); -2 -> -1 crosses the same mark the other way and
  // is forbidden, and no later road allows changes at all.
  Ids path = UnwrapIds(ShortestLanePath(graph, Id(1, 0, -1), {Id(2, 0, -2)},
                                        Eigen::Vector2d{42.0, -5.0}, options));
  ASSERT_GE(path.size(), 3U);
  EXPECT_EQ(path.front(), Id(1, 0, -1));
  EXPECT_EQ(path[1], Id(1, 0, -2));
  EXPECT_EQ(path.back(), Id(2, 0, -2));
  EXPECT_FALSE(ShortestLanePath(graph, Id(1, 0, -2), {Id(2, 0, -1)},
                                Eigen::Vector2d{42.0, -1.75}, options)
                   .has_value());
  // A prohibitive penalty does not change reachability, only cost.
  options.lane_change_penalty_m = 1e6;
  path = UnwrapIds(ShortestLanePath(graph, Id(1, 0, -1), {Id(2, 0, -2)},
                                    Eigen::Vector2d{42.0, -5.0}, options));
  EXPECT_EQ(path.back(), Id(2, 0, -2));
}

TEST(RoutePlanner, PlansThroughWaypointsInOrder) {
  const nuway_map::LaneGraph graph = BuildGraph(kCorridorXodr);
  const RoutePlannerOptions options;
  std::string error;
  const nuway_common::SE2 ego{2.0, -1.5, 0.0};
  // Three yaw-less waypoints: mid road 1, on the junction road, end of road 2.
  const nuway_common::Vector2dList waypoints = {
      {15.0, -1.75}, {35.0, -1.75}, {48.0, -1.75}};
  const RoutePlan plan =
      Unwrap(PlanRoute(graph, ego, waypoints, options, &error), error);
  EXPECT_EQ(plan.lane_ids,
            (Ids{Id(1, 0, -1), Id(5, 0, -1), Id(2, 0, -1), Id(2, 1, -1)}));
  EXPECT_EQ(plan.waypoint_lane_index, (std::vector<std::size_t>{0, 1, 3}));
  // A waypoint between the two eastbound lanes picks the nearer one (lane -2
  // at 1.0 m against lane -1 at 2.25 m) even though it costs a lane change.
  const RoutePlan plan2 =
      Unwrap(PlanRoute(graph, ego, {{47.0, -4.0}}, options, &error), error);
  EXPECT_EQ(plan2.lane_ids.back(), Id(2, 1, -2));
  EXPECT_EQ(plan2.lane_ids[1], Id(1, 0, -2));
  // Exactly between them (1.625 m each): the cheaper straight-on lane wins.
  const RoutePlan plan3 =
      Unwrap(PlanRoute(graph, ego, {{47.0, -3.375}}, options, &error), error);
  EXPECT_EQ(plan3.lane_ids.back(), Id(2, 1, -1));
  // Failures.
  EXPECT_FALSE(PlanRoute(graph, ego, {}, options, &error).has_value());
  EXPECT_EQ(error, "no waypoints");
  EXPECT_FALSE(PlanRoute(graph, nuway_common::SE2{2.0, 50.0, 0.0}, waypoints,
                         options, &error)
                   .has_value());
  EXPECT_EQ(error, "ego pose is on no lane");
  EXPECT_FALSE(
      PlanRoute(graph, ego, {{200.0, 200.0}}, options, &error).has_value());
  EXPECT_EQ(error, "waypoint 0 is on no lane");
  // Westbound waypoint from an eastbound ego on a corridor without a U-turn
  // (a tight tolerance keeps the eastbound lanes out of the goal set).
  RoutePlannerOptions tight = options;
  tight.waypoint_max_dist_m = 1.0;
  EXPECT_FALSE(PlanRoute(graph, ego, {{5.0, 1.75}}, tight, &error).has_value());
  EXPECT_EQ(error, "waypoint 0 is unreachable");
}

TEST(RoutePlanner, OverlappingGoalLanesAreChosenByTheNextWaypoint) {
  const nuway_map::LaneGraph graph = BuildGraph(kForkXodr);
  const RoutePlannerOptions options;
  std::string error;
  const nuway_common::SE2 ego{2.0, -1.75, 0.0};
  // Waypoint 0 sits where the two connecting lanes overlap (both at d = 0,
  // both 30 m away); waypoint 1 is near the end of road 3's lane, reachable
  // only through road 6. Per-segment planning could commit to road 5 here
  // and then find waypoint 1 unreachable.
  const RoutePlan plan = Unwrap(
      PlanRoute(graph, ego, {{33.0, -1.75}, {46.4, -8.2}}, options, &error),
      error);
  EXPECT_EQ(plan.lane_ids, (Ids{Id(1, 0, -1), Id(6, 0, -1), Id(3, 0, -1)}));
  EXPECT_EQ(plan.waypoint_lane_index, (std::vector<std::size_t>{1, 2}));
  // The same first waypoint followed by one on road 2 goes the other way.
  const RoutePlan plan2 = Unwrap(
      PlanRoute(graph, ego, {{33.0, -1.75}, {48.0, -1.75}}, options, &error),
      error);
  EXPECT_EQ(plan2.lane_ids, (Ids{Id(1, 0, -1), Id(5, 0, -1), Id(2, 0, -1)}));
  EXPECT_EQ(plan2.waypoint_lane_index, (std::vector<std::size_t>{1, 2}));
  // Two waypoints on one lane in driving order are both reached in place.
  const RoutePlan plan3 = Unwrap(
      PlanRoute(graph, ego, {{10.0, -1.75}, {20.0, -1.75}, {48.0, -1.75}},
                options, &error),
      error);
  EXPECT_EQ(plan3.lane_ids, (Ids{Id(1, 0, -1), Id(5, 0, -1), Id(2, 0, -1)}));
  EXPECT_EQ(plan3.waypoint_lane_index, (std::vector<std::size_t>{0, 0, 2}));
  // Reversed on the same lane with no loop back: unreachable.
  EXPECT_FALSE(
      PlanRoute(graph, ego, {{20.0, -1.75}, {10.0, -1.75}}, options, &error)
          .has_value());
  EXPECT_EQ(error, "waypoint 1 lies behind on the current lane");
}

TEST(RoutePlanner, WaypointBehindOnTheCurrentLaneIsNotReached) {
  const nuway_map::LaneGraph graph = BuildGraph(kCorridorXodr);
  RoutePlannerOptions options;
  options.waypoint_max_dist_m = 1.0;
  std::string error;
  // Ego at x=20 on lane -1; a waypoint at x=5 on the same lane is behind and
  // there is no loop back, so the plan fails.
  EXPECT_FALSE(PlanRoute(graph, nuway_common::SE2{20.0, -1.75, 0.0},
                         {{5.0, -1.75}}, options, &error)
                   .has_value());
  EXPECT_EQ(error, "waypoint 0 lies behind on the current lane");
  // Just behind (within passed_tolerance_m): the ego is already there, so
  // the plan continues from the current lane without a loop.
  const RoutePlan near_plan =
      Unwrap(PlanRoute(graph, nuway_common::SE2{20.0, -1.75, 0.0},
                       {{18.0, -1.75}, {48.0, -1.75}}, options, &error),
             error);
  EXPECT_EQ(near_plan.lane_ids,
            (Ids{Id(1, 0, -1), Id(5, 0, -1), Id(2, 0, -1), Id(2, 1, -1)}));
  EXPECT_EQ(near_plan.waypoint_lane_index, (std::vector<std::size_t>{0, 3}));
  // On the ring the lane loops onto itself, so a waypoint behind is reached
  // by going around: ego a quarter turn in on lane -1 (radius 31.75), the
  // waypoint at the lane's start.
  const nuway_map::LaneGraph ring = BuildGraph(kCircleXodr);
  const nuway_common::SE2 ego{0.0, 31.75, 3.14159265358979323846};
  const RoutePlan plan =
      Unwrap(PlanRoute(ring, ego, {{31.75, 0.0}}, options, &error), error);
  EXPECT_EQ(plan.lane_ids, (Ids{Id(7, 0, -1), Id(7, 0, -1)}));
}

}  // namespace
}  // namespace nuway_route
