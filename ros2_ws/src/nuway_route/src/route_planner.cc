// A* searches behind route_planner.h. ShortestLanePath runs over lane ids to
// one goal set (the single-waypoint form); PlanRoute runs over the joint
// state (lane, waypoints reached) to thread every waypoint in order. Both
// share the machinery described here.
//
// A* in brief. Every state carries g, the cheapest known cost from the
// start, and f = g + h, where h is a heuristic estimate of the cost still to
// pay. The open list is a min-heap on f: the state popped next is the one
// that looks cheapest overall. Expanding a popped state *relaxes* each of
// its outgoing edges: if start -> state -> next is cheaper than the best g
// recorded for next so far, record the new g, note state as next's parent
// and push next with its new f. A popped state joins the closed set and is
// never expanded again. When the popped state is a goal, the path is rebuilt
// by following parents back to the start and reversing.
//
// Optimality. h is admissible when it never overestimates the true
// remaining cost. Then the first goal popped is optimal: its f equals its g
// (h is 0 on a goal), every state still on the heap has f at least as
// large, and f is a lower bound on the cost of any path through that state,
// so no other path can be cheaper. h = 0 turns A* into Dijkstra's
// algorithm; a tighter admissible h expands fewer states while keeping the
// guarantee. RemainingLowerBound is the argument for this graph. Strictly,
// a closed set that never reopens a state also wants h *consistent*
// (h(u) <= cost(u, v) + h(v), so f never decreases along an edge); the bound
// below is Euclidean, hence consistent for the distance actually driven, and
// only the charging of a successor edge by the lane being *left* rather than
// entered can dent that. The route tests pin the results on the maps used.
//
// Lazy closed-set check. A state may be pushed several times, once per
// improvement of its g, because std::priority_queue has no decrease-key.
// The stale copies stay on the heap with their larger f; when one is popped
// its state is already closed and it is skipped. That costs a few extra
// heap entries and saves a heap that supports removal.
#include "nuway_route/route_planner.h"

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

// One open-list entry of the single-goal search: f = g + h and the lane.
// operator> orders by f, so std::greater<> makes the priority_queue (a
// max-heap by default) a min-heap on f.
struct Open {
  double f = 0.0;
  double g = 0.0;
  std::uint32_t lane_id = 0;
  bool operator>(const Open& other) const { return f > other.f; }
};

// Admissible lower bound on the cost still to pay from `lane` to a goal
// lane holding `goal_xy`. Costs are charged when a lane is *left* (its
// length) or changed (a flat penalty); the goal lane itself is never charged
// and is reached the moment it is expanded. Hence:
//   - on a goal lane, or one lateral change away from one, nothing is
//     certain to be paid: 0;
//   - otherwise the chain of lanes still to be traversed covers at least the
//     straight distance from this lane's end to wherever the goal lane is
//     entered, which lies within `slack_m` (the longest goal lane plus a
//     lane width) of the waypoint.
// Why that is a lower bound: every metre driven after this lane is charged,
// and the lanes driven form a polyline from this lane's end to the goal
// lane's entry point, no shorter than the straight line between them. The
// entry point is unknown, but it lies on a goal lane within `slack_m` of the
// waypoint, so by the triangle inequality the straight line is at least
// |end - waypoint| - slack_m. The straight-line distance from the lane
// *entry* to the waypoint, used before, overestimated on the goal lane (its
// own length is free) and gave valid but not always shortest routes.
double RemainingLowerBound(const nuway_map::Lane& lane,
                           const std::unordered_set<std::uint32_t>& goal_ids,
                           const Eigen::Vector2d& goal_xy, double slack_m) {
  if (goal_ids.count(lane.id) != 0U ||
      (lane.left_neighbor != 0 && goal_ids.count(lane.left_neighbor) != 0U) ||
      (lane.right_neighbor != 0 && goal_ids.count(lane.right_neighbor) != 0U)) {
    return 0.0;
  }
  const double to_goal = (lane.centerline.back().head<2>() - goal_xy).norm();
  return std::max(0.0, to_goal - slack_m);
}

// Longest goal lane plus a lane width: how far from the waypoint the goal
// lane may be entered. The waypoint lies somewhere along a goal lane (so
// its entry is at most that lane's length away) and up to a lane width
// beside it (the goal set admits lanes within goal_slack_m of the nearest).
double GoalSlackM(const nuway_map::LaneGraph& graph,
                  const std::unordered_set<std::uint32_t>& goal_ids) {
  double longest = 0.0;
  for (const std::uint32_t id : goal_ids) {
    const nuway_map::Lane* lane = graph.lane(id);
    if (lane != nullptr) {
      longest = std::max(longest, lane->length_m);
    }
  }
  return longest + 5.0;
}

}  // namespace

// Textbook A* over lane ids (see the file header). The goal test is made on
// *pop*, not on push: a goal pushed by some edge may still be reached more
// cheaply by another, and only the pop order proves it is the cheapest.
std::optional<std::vector<std::uint32_t>> ShortestLanePath(
    const nuway_map::LaneGraph& graph, std::uint32_t start,
    const std::vector<std::uint32_t>& goals, const Eigen::Vector2d& goal_xy,
    const RoutePlannerOptions& options) {
  const std::unordered_set<std::uint32_t> goal_set(goals.begin(), goals.end());
  if (graph.lane(start) == nullptr || goal_set.empty()) {
    return std::nullopt;
  }
  const double slack_m = GoalSlackM(graph, goal_set);
  const auto heuristic = [&](const nuway_map::Lane& lane) {
    return RemainingLowerBound(lane, goal_set, goal_xy, slack_m);
  };
  std::priority_queue<Open, std::vector<Open>, std::greater<>> open;
  std::unordered_map<std::uint32_t, double> best_g;         // cheapest g so far
  std::unordered_map<std::uint32_t, std::uint32_t> parent;  // for rebuild
  std::unordered_set<std::uint32_t> closed;
  open.push(Open{heuristic(*graph.lane(start)), 0.0, start});
  best_g[start] = 0.0;
  while (!open.empty()) {
    const Open current = open.top();
    open.pop();
    if (closed.count(current.lane_id) != 0U) {
      continue;  // stale duplicate: a cheaper copy was popped earlier
    }
    closed.insert(current.lane_id);
    if (goal_set.count(current.lane_id) != 0U) {
      // Rebuild the path by walking the parent links back to the start.
      std::vector<std::uint32_t> path{current.lane_id};
      while (path.back() != start) {
        path.push_back(parent.at(path.back()));
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
    const nuway_map::Lane* lane = graph.lane(current.lane_id);
    // Relaxation: record next_id's new best g and parent when reaching it
    // through the current lane is cheaper than anything found before.
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
      open.push(Open{g + heuristic(*next), g, next_id});
    };
    // Leaving the lane forward costs its length; sideways, the flat penalty.
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
  return std::nullopt;  // open list exhausted: no goal lane is reachable
}

namespace {

// A goal lane for one waypoint and where along it the waypoint lies.
struct Goal {
  std::uint32_t lane_id = 0;
  double s = 0.0;  // arc length of the waypoint's projection on the lane
};

// goals[i] is the goal set of waypoint i.
using GoalSets = std::vector<std::vector<Goal>>;

// Linear lookup of `lane_id` in one waypoint's goal set (a few lanes).
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
// `stage` is what threads the waypoints: stage n (all reached) is the goal,
// and a lane can be revisited at a higher stage, which is how a waypoint
// behind the ego is reached by looping back to its lane.
struct JointOpen {
  double f = 0.0;
  double g = 0.0;
  std::uint32_t lane_id = 0;
  std::size_t stage = 0;
  double pos_s = 0.0;
  bool operator>(const JointOpen& other) const { return f > other.f; }
};

// Packs (lane, stage) into one 64-bit hash key: stage in the high word, lane
// id in the low word.
std::uint64_t StateKey(std::uint32_t lane_id, std::size_t stage) {
  return (static_cast<std::uint64_t>(stage) << 32U) | lane_id;
}

// Lane id of a state key.
std::uint32_t KeyLane(std::uint64_t key) {
  return static_cast<std::uint32_t>(key & 0xffffffffU);
}

// Stage of a state key.
std::size_t KeyStage(std::uint64_t key) {
  return static_cast<std::size_t>(key >> 32U);
}

// Counts off every waypoint that lies ahead (or within passed_tolerance_m
// behind) on `lane_id` from `*pos_s`, in order, moving `*pos_s` to the last
// one. Starting at `stage`, each waypoint whose goal set holds this lane and
// whose s is not too far behind the current position is reached, and the
// position advances to it (never back, so a tolerated waypoint slightly
// behind does not rewind the progress). The loop stops at the first
// waypoint that is on another lane or too far behind, and the returned
// stage is the count reached after entering the lane at `*pos_s`; several
// waypoints on one lane are thus counted in road order. `behind_seen`
// records the stage whose waypoint sits on this lane but too far behind to
// count, for the failure message. The tolerance covers the harness resetting
// the hero onto waypoint 0 (a little past it in practice).
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

// Joint A* over (lane, stage). The edge costs and the relaxation are those
// of ShortestLanePath; the difference is that entering a lane also counts
// off the waypoints reached on it (AdvanceInPlace), which may raise the
// stage, and that the heuristic bounds only the leg to the *next* waypoint
// (an admissible bound for the whole remainder, since the later legs cost
// at least 0).
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
  // 1. Start state: the lane under the ego, chosen with its heading so that
  // the opposite direction of the same road is not picked.
  const std::optional<nuway_map::LaneQuery> start =
      graph.NearestLane(ego.x, ego.y, ego.yaw, options.ego_max_dist_m);
  if (!start.has_value()) {
    return fail("ego pose is on no lane");
  }
  // 2. Goal set per waypoint: the lanes about as near to it as the nearest
  // one. LanesNear returns candidates sorted by |d|, so the scan stops at
  // the first one beyond the slack.
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
  // 3. A* over (lane, waypoints reached) so that a waypoint's goal lane is
  // chosen by the whole remaining route, not by that segment alone: at a
  // junction several overlapping lanes sit on the waypoint and only some
  // lead on towards the next one.
  const std::size_t n = waypoints.size();
  std::vector<std::unordered_set<std::uint32_t>> goal_ids(n);
  std::vector<double> slack_m(n, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    for (const Goal& g : goals[i]) {
      goal_ids[i].insert(g.lane_id);
    }
    slack_m[i] = GoalSlackM(graph, goal_ids[i]);
  }
  // h of a state is the lower bound for its next waypoint; 0 once all are
  // reached.
  const auto heuristic = [&](const nuway_map::Lane& lane, std::size_t stage) {
    return stage >= n ? 0.0
                      : RemainingLowerBound(lane, goal_ids[stage],
                                            waypoints[stage], slack_m[stage]);
  };
  std::vector<bool> behind_seen(n, false);
  std::priority_queue<JointOpen, std::vector<JointOpen>, std::greater<>> open;
  std::unordered_map<std::uint64_t, double> best_g;
  std::unordered_map<std::uint64_t, std::uint64_t> parent;
  std::unordered_set<std::uint64_t> closed;
  // The start lane may already hold the first waypoints (the harness resets
  // the hero onto waypoint 0), so the start state can begin past stage 0.
  double start_pos = start->s;
  const std::size_t start_stage = AdvanceInPlace(
      goals, start->lane_id, 0, &start_pos, options, &behind_seen);
  const std::uint64_t start_key = StateKey(start->lane_id, start_stage);
  open.push(JointOpen{heuristic(*graph.lane(start->lane_id), start_stage), 0.0,
                      start->lane_id, start_stage, start_pos});
  best_g[start_key] = 0.0;
  std::size_t deepest = start_stage;  // furthest stage any closed state hit
  std::optional<std::uint64_t> goal_key;
  while (!open.empty()) {
    const JointOpen current = open.top();
    open.pop();
    const std::uint64_t key = StateKey(current.lane_id, current.stage);
    if (closed.count(key) != 0U) {
      continue;  // stale duplicate (lazy closed-set check)
    }
    closed.insert(key);
    deepest = std::max(deepest, current.stage);
    if (current.stage == n) {
      goal_key = key;  // every waypoint reached: first pop is the cheapest
      break;
    }
    const nuway_map::Lane* lane = graph.lane(current.lane_id);
    // Relaxation with the stage advanced by the waypoints met on entering
    // `next_id` at arc length `entry_s`. The (lane, stage) key after that
    // advance is the state relaxed, so one lane can appear at several
    // stages.
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
    // A successor is entered at its start (s = 0) for the length of the
    // lane left.
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
  // 4. Failure diagnosis: the first waypoint no state got past is the one
  // to blame. When its only goal lane was seen with the waypoint behind, the
  // lane has no loop back to it.
  if (!goal_key.has_value()) {
    if (goals[deepest].size() == 1 && behind_seen[deepest]) {
      return fail("waypoint " + std::to_string(deepest) +
                  " lies behind on the current lane");
    }
    return fail("waypoint " + std::to_string(deepest) + " is unreachable");
  }
  // 5. Rebuild the state chain from the parents, then unpack it: each state
  // contributes its lane, and the stages it advanced past its predecessor
  // are the waypoints reached on that lane.
  std::vector<std::uint64_t> chain{*goal_key};
  while (chain.back() != start_key) {
    chain.push_back(parent.at(chain.back()));
  }
  std::reverse(chain.begin(), chain.end());
  RoutePlan plan;
  plan.start_s_m = start->s;
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
