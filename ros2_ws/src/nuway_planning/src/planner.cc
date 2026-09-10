#include "nuway_planning/planner.h"

#include <algorithm>
#include <cstddef>
#include <map>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nuway_common/frenet.h>

namespace nuway_planning {

Planner::Planner(PlannerOptions options)
    : options_(std::move(options)),
      sampler_(options_.lattice, options_.limits),
      checker_(options_.collision),
      refiner_(options_.refiner, options_.limits, options_.collision),
      selector_(options_.weights) {}

void Planner::Reset() {
  refiner_.Reset();
  s_hint_.reset();
  previous_.reset();
}

PlanResult Planner::Plan(const SceneInput& in,
                         const std::optional<BehaviorOutput>& decision) {
  PlanResult out;
  refiner_.OnPlanningTick();
  if (in.route == nullptr || in.route->empty()) {
    out.no_input = true;
    out.message = "no route line";
    return out;
  }
  const RouteLine& route = *in.route;
  const std::optional<nuway_common::FrenetState> ego = EgoFrenetState(
      in.ego, route, options_.lattice, options_.limits.wheelbase_m, s_hint_);
  if (!ego.has_value()) {
    out.no_input = true;
    out.message = "ego off the route line";
    s_hint_.reset();
    return out;
  }
  s_hint_ = ego->s;
  out.ego_s = ego->s;
  out.ego_d = ego->d;

  // 1. Sample: the decision's lattice, or the injected pair alone.
  out.used_decision = decision.has_value() && decision->reason != "no_input";
  std::vector<Candidate> set = out.used_decision
                                   ? sampler_.Sample(in, *ego, *decision)
                                   : sampler_.SampleInjected(in, *ego, 0);
  out.sampled = static_cast<int>(set.size());

  // 2. Filter: kinematics and bounds, then collision (normal candidates
  // only; the injected pair is scored but never rejected, §3.3).
  sampler_.Filter(route, &set);
  std::vector<CollisionResult> collisions(set.size());
  for (std::size_t i = 0; i < set.size(); ++i) {
    Candidate& c = set[i];
    if (!c.injected && !c.feasible()) {
      continue;
    }
    collisions[i] = checker_.Check(c.trajectory, in.agents, in.predictions);
    if (!c.injected && collisions[i].collides()) {
      c.reject = "collision";
    }
  }

  // 3. Score everything that survived (the pre-QP rule cost).
  SelectorContext ctx;
  ctx.route = &route;
  ctx.ego_s = ego->s;
  ctx.v_limit_mps = route.SpeedLimitAt(ego->s);
  const LateralBounds bounds = route.BoundsAt(ego->s);
  ctx.lane_half_width_m = 0.5 * (bounds.left_m + bounds.right_m);
  if (out.used_decision) {
    ctx.target_speed_mps = decision->target_speed_mps;
    if (decision->stop_s >= 0.0 &&
        (decision->longitudinal == Longitudinal::kStop ||
         decision->longitudinal == Longitudinal::kYield)) {
      ctx.stop_s = decision->stop_s;
    }
  }
  ctx.previous = previous_.has_value() ? &*previous_ : nullptr;
  ctx.previous_age_s = options_.tick_period_s;
  std::vector<std::size_t> feasible;
  for (std::size_t i = 0; i < set.size(); ++i) {
    if (set[i].injected || set[i].feasible()) {
      selector_.Score(collisions[i], ctx, &set[i]);
    }
    if (!set[i].injected && set[i].feasible()) {
      feasible.push_back(i);
    }
  }
  out.feasible = static_cast<int>(feasible.size());

  // 4. Refine the K cheapest normals; re-check and re-score their new
  // shapes (the QP moved them, so the pre-QP collision result is stale).
  std::stable_sort(feasible.begin(), feasible.end(),
                   [&set](std::size_t a, std::size_t b) {
                     return set[a].cost < set[b].cost;
                   });
  const std::size_t k = std::min(
      feasible.size(), static_cast<std::size_t>(std::max(0, options_.top_k)));
  std::string qp_messages;
  for (std::size_t r = 0; r < k; ++r) {
    Candidate& c = set[feasible[r]];
    const RefineOutcome outcome = refiner_.Refine(in, *ego, &c);
    if (outcome.refined()) {
      ++out.refined;
      collisions[feasible[r]] =
          checker_.Check(c.trajectory, in.agents, in.predictions);
      if (collisions[feasible[r]].collides()) {
        // The refined shape collides where the lattice one did not: the
        // filter's rule still applies, so it cannot be selected.
        c.reject = "collision";
      }
    } else {
      ++out.qp_failed;
      // A failed QP leaves the lattice shape: still selectable (§3.5),
      // marked refined so Select() considers it, with the penalty.
      c.refined = true;
      if (!qp_messages.empty()) {
        qp_messages += "; ";
      }
      qp_messages +=
          "candidate " + std::to_string(c.id) + ": " + outcome.message;
    }
    if (c.qp_relaxed) {
      ++out.qp_relaxed;
    }
    selector_.Score(collisions[feasible[r]], ctx, &c);
  }

  // 5. Select among the refined and the injected; assemble the published
  // list: those first by cost, then the best of the rest.
  std::vector<std::size_t> order(set.size());
  std::iota(order.begin(), order.end(), std::size_t{0});
  const auto rank = [&set](std::size_t i) {
    const Candidate& c = set[i];
    if (c.injected || c.refined) {
      return 0;
    }
    return c.feasible() ? 1 : 2;
  };
  std::stable_sort(order.begin(), order.end(),
                   [&](std::size_t a, std::size_t b) {
                     const int ra = rank(a);
                     const int rb = rank(b);
                     if (ra != rb) {
                       return ra < rb;
                     }
                     return set[a].cost < set[b].cost;
                   });
  std::string reject_summary;
  // Why the normal candidates were rejected, so a stop that came from
  // the injected pair can be read off the diag (the first drives of task
  // 9 were debugged from exactly this).
  std::map<std::string, int> rejects;
  for (const Candidate& c : set) {
    if (!c.injected && !c.reject.empty()) {
      ++rejects[c.reject];
    }
  }
  if (!rejects.empty()) {
    reject_summary = "; rejected:";
    for (const auto& [reason, count] : rejects) {
      reject_summary += " " + reason + " " + std::to_string(count);
    }
  }
  const auto limit =
      static_cast<std::size_t>(std::max(2, options_.publish_max));
  for (const std::size_t i : order) {
    if (rank(i) == 2 || out.candidates.size() >= limit) {
      break;
    }
    out.candidates.push_back(std::move(set[i]));
  }
  out.selected_index = RuleSelector::Select(out.candidates);
  if (out.selected_index >= 0) {
    previous_ =
        out.candidates[static_cast<std::size_t>(out.selected_index)].trajectory;
  }
  out.message = (out.used_decision ? "decision " + decision->reason
                                   : std::string("no decision")) +
                ": " + std::to_string(out.sampled) + " sampled, " +
                std::to_string(out.feasible) + " feasible, " +
                std::to_string(out.refined) + " refined";
  if (!qp_messages.empty()) {
    out.message += "; qp failed: " + qp_messages;
  }
  out.message += reject_summary;
  return out;
}

}  // namespace nuway_planning
