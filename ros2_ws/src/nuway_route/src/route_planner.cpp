#include "nuway_route/route_planner.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace nuway_route {
namespace {

struct Open {
  double f = 0.0;
  double g = 0.0;
  std::uint32_t lane_id = 0;
  bool operator>(const Open& other) const { return f > other.f; }
};

double Heuristic(const nuway_map::Lane& lane, const Eigen::Vector2d& goal_xy) {
  // Straight-line distance from the lane's entry: admissible because every
  // edge out of the lane is charged at least the lane's own length.
  return (lane.centerline.front().head<2>() - goal_xy).norm();
}

}  // namespace

std::optional<std::vector<std::uint32_t>> ShortestLanePath(
    const nuway_map::LaneGraph& graph, std::uint32_t start,
    const std::vector<std::uint32_t>& goals, const Eigen::Vector2d& goal_xy,
    const RoutePlannerOptions& options) {
  const std::unordered_set<std::uint32_t> goal_set(goals.begin(), goals.end());
  if (graph.lane(start) == nullptr || goal_set.empty()) {
    return std::nullopt;
  }
  std::priority_queue<Open, std::vector<Open>, std::greater<>> open;
  std::unordered_map<std::uint32_t, double> best_g;
  std::unordered_map<std::uint32_t, std::uint32_t> parent;
  std::unordered_set<std::uint32_t> closed;
  open.push(Open{Heuristic(*graph.lane(start), goal_xy), 0.0, start});
  best_g[start] = 0.0;
  while (!open.empty()) {
    const Open current = open.top();
    open.pop();
    if (closed.count(current.lane_id) != 0U) {
      continue;
    }
    closed.insert(current.lane_id);
    if (goal_set.count(current.lane_id) != 0U) {
      std::vector<std::uint32_t> path{current.lane_id};
      while (path.back() != start) {
        path.push_back(parent.at(path.back()));
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
    const nuway_map::Lane* lane = graph.lane(current.lane_id);
    const auto relax = [&](std::uint32_t next_id, double edge_cost) {
      const nuway_map::Lane* next = graph.lane(next_id);
      if (next == nullptr || closed.count(next_id) != 0U) {
        return;
      }
      const double g = current.g + edge_cost;
      const auto it = best_g.find(next_id);
      if (it != best_g.end() && it->second <= g) {
        return;
      }
      best_g[next_id] = g;
      parent[next_id] = current.lane_id;
      open.push(Open{g + Heuristic(*next, goal_xy), g, next_id});
    };
    for (const std::uint32_t succ : lane->successors) {
      relax(succ, lane->length_m);
    }
    if (lane->left_neighbor != 0 && lane->left_change_allowed) {
      relax(lane->left_neighbor, options.lane_change_penalty_m);
    }
    if (lane->right_neighbor != 0 && lane->right_change_allowed) {
      relax(lane->right_neighbor, options.lane_change_penalty_m);
    }
  }
  return std::nullopt;
}

std::optional<RoutePlan> PlanRoute(const nuway_map::LaneGraph& graph,
                                   const nuway_common::SE2& ego,
                                   const nuway_common::Vector2dList& waypoints,
                                   const RoutePlannerOptions& options,
                                   std::string* error) {
  const auto fail = [&](const std::string& why) -> std::optional<RoutePlan> {
    if (error != nullptr) {
      *error = why;
    }
    return std::nullopt;
  };
  if (waypoints.empty()) {
    return fail("no waypoints");
  }
  const std::optional<nuway_map::LaneQuery> start =
      graph.NearestLane(ego.x, ego.y, ego.yaw, options.ego_max_dist_m);
  if (!start.has_value()) {
    return fail("ego pose is on no lane");
  }
  RoutePlan plan;
  plan.lane_ids.push_back(start->lane_id);
  double start_s = start->s;
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const Eigen::Vector2d& wp = waypoints[i];
    const std::vector<nuway_map::LaneQuery> near =
        graph.LanesNear(wp.x(), wp.y(), options.waypoint_max_dist_m);
    if (near.empty()) {
      return fail("waypoint " + std::to_string(i) + " is on no lane");
    }
    const std::uint32_t current = plan.lane_ids.back();
    std::vector<std::uint32_t> goals;
    bool current_behind = false;
    for (const nuway_map::LaneQuery& q : near) {
      if (std::abs(q.d) > std::abs(near.front().d) + options.goal_slack_m) {
        break;  // sorted by |d|
      }
      // The current lane counts only if the waypoint is still ahead.
      if (q.lane_id == current && q.s + 1e-6 < start_s) {
        current_behind = true;
        continue;
      }
      goals.push_back(q.lane_id);
    }
    std::optional<std::vector<std::uint32_t>> segment;
    if (!goals.empty()) {
      segment = ShortestLanePath(graph, current, goals, wp, options);
    }
    if (!segment.has_value() && current_behind) {
      // Loop back onto the current lane through its successors.
      for (const std::uint32_t succ : graph.Successors(current)) {
        std::optional<std::vector<std::uint32_t>> loop =
            ShortestLanePath(graph, succ, {current}, wp, options);
        if (loop.has_value() &&
            (!segment.has_value() || loop->size() + 1 < segment->size())) {
          loop->insert(loop->begin(), current);
          segment = std::move(loop);
        }
      }
      if (!segment.has_value()) {
        return fail("waypoint " + std::to_string(i) +
                    " lies behind on the current lane");
      }
    }
    if (!segment.has_value()) {
      return fail("waypoint " + std::to_string(i) + " is unreachable");
    }
    for (std::size_t j = 1; j < segment->size(); ++j) {
      plan.lane_ids.push_back((*segment)[j]);
    }
    plan.waypoint_lane_index.push_back(plan.lane_ids.size() - 1);
    // Progress along the reached lane, for the "still ahead" test above.
    start_s = 0.0;
    for (const nuway_map::LaneQuery& q : near) {
      if (q.lane_id == plan.lane_ids.back()) {
        start_s = q.s;
        break;
      }
    }
  }
  return plan;
}

}  // namespace nuway_route
