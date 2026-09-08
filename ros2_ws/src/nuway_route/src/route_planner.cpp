#include "nuway_route/route_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
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

namespace {

// A goal lane for one waypoint and where along it the waypoint lies.
struct Goal {
  std::uint32_t lane_id = 0;
  double s = 0.0;
};

using GoalSets = std::vector<std::vector<Goal>>;

const Goal* FindGoal(const std::vector<Goal>& goals, std::uint32_t lane_id) {
  for (const Goal& g : goals) {
    if (g.lane_id == lane_id) {
      return &g;
    }
  }
  return nullptr;
}

// Search state: on `lane_id` at progress `pos_s` with `stage` waypoints
// reached. The (lane, stage) pair identifies the state; pos_s rides along.
struct JointOpen {
  double f = 0.0;
  double g = 0.0;
  std::uint32_t lane_id = 0;
  std::size_t stage = 0;
  double pos_s = 0.0;
  bool operator>(const JointOpen& other) const { return f > other.f; }
};

std::uint64_t StateKey(std::uint32_t lane_id, std::size_t stage) {
  return (static_cast<std::uint64_t>(stage) << 32U) | lane_id;
}

std::uint32_t KeyLane(std::uint64_t key) {
  return static_cast<std::uint32_t>(key & 0xffffffffU);
}

std::size_t KeyStage(std::uint64_t key) {
  return static_cast<std::size_t>(key >> 32U);
}

// Counts off every waypoint that lies ahead (or within passed_tolerance_m
// behind) on `lane_id` from `*pos_s`, in order, moving `*pos_s` to the last
// one. `behind_seen` records the stage whose waypoint sits on this lane but
// too far behind to count.
std::size_t AdvanceInPlace(const GoalSets& goals, std::uint32_t lane_id,
                           std::size_t stage, double* pos_s,
                           const RoutePlannerOptions& options,
                           std::vector<bool>* behind_seen) {
  while (stage < goals.size()) {
    const Goal* goal = FindGoal(goals[stage], lane_id);
    if (goal == nullptr) {
      break;
    }
    if (goal->s + options.passed_tolerance_m < *pos_s) {
      (*behind_seen)[stage] = true;
      break;
    }
    *pos_s = std::max(*pos_s, goal->s);
    ++stage;
  }
  return stage;
}

}  // namespace

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
  // Goal set per waypoint: the lanes about as near to it as the nearest one.
  GoalSets goals(waypoints.size());
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const std::vector<nuway_map::LaneQuery> near = graph.LanesNear(
        waypoints[i].x(), waypoints[i].y(), options.waypoint_max_dist_m);
    if (near.empty()) {
      return fail("waypoint " + std::to_string(i) + " is on no lane");
    }
    for (const nuway_map::LaneQuery& q : near) {
      if (std::abs(q.d) > std::abs(near.front().d) + options.goal_slack_m) {
        break;  // sorted by |d|
      }
      goals[i].push_back(Goal{q.lane_id, q.s});
    }
  }
  // A* over (lane, waypoints reached) so that a waypoint's goal lane is
  // chosen by the whole remaining route, not by that segment alone: at a
  // junction several overlapping lanes sit on the waypoint and only some
  // lead on towards the next one.
  const std::size_t n = waypoints.size();
  const auto heuristic = [&](const nuway_map::Lane& lane, std::size_t stage) {
    return stage >= n ? 0.0 : Heuristic(lane, waypoints[stage]);
  };
  std::vector<bool> behind_seen(n, false);
  std::priority_queue<JointOpen, std::vector<JointOpen>, std::greater<>> open;
  std::unordered_map<std::uint64_t, double> best_g;
  std::unordered_map<std::uint64_t, std::uint64_t> parent;
  std::unordered_set<std::uint64_t> closed;
  double start_pos = start->s;
  const std::size_t start_stage = AdvanceInPlace(
      goals, start->lane_id, 0, &start_pos, options, &behind_seen);
  const std::uint64_t start_key = StateKey(start->lane_id, start_stage);
  open.push(JointOpen{heuristic(*graph.lane(start->lane_id), start_stage), 0.0,
                      start->lane_id, start_stage, start_pos});
  best_g[start_key] = 0.0;
  std::size_t deepest = start_stage;
  std::optional<std::uint64_t> goal_key;
  while (!open.empty()) {
    const JointOpen current = open.top();
    open.pop();
    const std::uint64_t key = StateKey(current.lane_id, current.stage);
    if (closed.count(key) != 0U) {
      continue;
    }
    closed.insert(key);
    deepest = std::max(deepest, current.stage);
    if (current.stage == n) {
      goal_key = key;
      break;
    }
    const nuway_map::Lane* lane = graph.lane(current.lane_id);
    const auto relax = [&](std::uint32_t next_id, double edge_cost,
                           double entry_s) {
      const nuway_map::Lane* next = graph.lane(next_id);
      if (next == nullptr) {
        return;
      }
      double pos = entry_s;
      const std::size_t stage = AdvanceInPlace(goals, next_id, current.stage,
                                               &pos, options, &behind_seen);
      const std::uint64_t next_key = StateKey(next_id, stage);
      if (closed.count(next_key) != 0U) {
        return;
      }
      const double g = current.g + edge_cost;
      const auto it = best_g.find(next_key);
      if (it != best_g.end() && it->second <= g) {
        return;
      }
      best_g[next_key] = g;
      parent[next_key] = key;
      open.push(JointOpen{g + heuristic(*next, stage), g, next_id, stage, pos});
    };
    for (const std::uint32_t succ : lane->successors) {
      relax(succ, lane->length_m, 0.0);
    }
    // A lane change keeps the progress along the road.
    if (lane->left_neighbor != 0 && lane->left_change_allowed) {
      relax(lane->left_neighbor, options.lane_change_penalty_m, current.pos_s);
    }
    if (lane->right_neighbor != 0 && lane->right_change_allowed) {
      relax(lane->right_neighbor, options.lane_change_penalty_m, current.pos_s);
    }
  }
  if (!goal_key.has_value()) {
    if (goals[deepest].size() == 1 && behind_seen[deepest]) {
      return fail("waypoint " + std::to_string(deepest) +
                  " lies behind on the current lane");
    }
    return fail("waypoint " + std::to_string(deepest) + " is unreachable");
  }
  std::vector<std::uint64_t> chain{*goal_key};
  while (chain.back() != start_key) {
    chain.push_back(parent.at(chain.back()));
  }
  std::reverse(chain.begin(), chain.end());
  RoutePlan plan;
  std::size_t stage = 0;
  for (const std::uint64_t k : chain) {
    plan.lane_ids.push_back(KeyLane(k));
    for (; stage < KeyStage(k); ++stage) {
      plan.waypoint_lane_index.push_back(plan.lane_ids.size() - 1);
    }
  }
  return plan;
}

}  // namespace nuway_route
