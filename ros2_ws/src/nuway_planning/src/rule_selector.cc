#include "nuway_planning/rule_selector.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <nuway_common/frenet.h>
#include <nuway_common/trajectory.h>

namespace nuway_planning {

RuleSelector::RuleSelector(SelectorWeights weights) : weights_(weights) {}

void RuleSelector::Score(const CollisionResult& collision,
                         const SelectorContext& ctx, Candidate* c) const {
  std::array<double, kNumCostTerms> terms{};
  const nuway_common::Trajectory& traj = c->trajectory;
  const std::size_t n = traj.size();
  const double v_limit = std::max(ctx.v_limit_mps, 0.1);
  const double half_width = std::max(ctx.lane_half_width_m, 0.1);

  // collision: the weighted fraction of samples that collide.
  terms[0] = collision.cost;
  // ttc: how soon the first collision comes, 0 without one.
  terms[1] =
      std::isfinite(collision.min_ttc_s)
          ? std::max(0.0, 1.0 - (collision.min_ttc_s / weights_.ttc_horizon_s))
          : 0.0;
  // proximity: closeness to any agent, averaged over the check times.
  if (!collision.min_distance_m.empty()) {
    double sum = 0.0;
    for (const double d : collision.min_distance_m) {
      sum += std::max(0.0, 1.0 - (d / weights_.proximity_range_m));
    }
    terms[2] = sum / static_cast<double>(collision.min_distance_m.size());
  }
  if (n > 0 && c->frenet.size() == n) {
    const double horizon_s =
        nuway_common::kTrajectoryDtS * static_cast<double>(n - 1);
    // progress: arc length covered over what the limit would allow
    // (negative: more progress is cheaper).
    terms[3] = -(c->frenet.back().s - ctx.ego_s) /
               (v_limit * std::max(horizon_s, 0.1));
    double speed_dev = 0.0;
    double lateral_dev = 0.0;
    double comfort = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      const nuway_common::TrajectoryPoint& p = traj[i];
      speed_dev += std::abs(p.v - ctx.target_speed_mps) / v_limit;
      lateral_dev += std::abs(c->frenet[i].d) / half_width;
      const double a_lat = p.v * p.v * p.kappa / weights_.a_lat_norm_mps2;
      double jerk = 0.0;
      if (i + 1 < n) {
        const double dt = traj[i + 1].t - p.t;
        jerk = dt > 1e-9 ? (traj[i + 1].a - p.a) / dt : 0.0;
      }
      jerk /= weights_.jerk_norm_mps3;
      comfort += (a_lat * a_lat) + (jerk * jerk);
    }
    terms[4] = speed_dev / static_cast<double>(n);
    terms[5] = lateral_dev / static_cast<double>(n);
    terms[6] = comfort / static_cast<double>(n);
    // rule: passing a stop line, or leaving the drivable width.
    bool violates =
        ctx.stop_s.has_value() &&
        c->frenet.back().s > *ctx.stop_s + weights_.stop_line_tolerance_m;
    if (ctx.route != nullptr) {
      for (const nuway_common::FrenetState& f : c->frenet) {
        const LateralBounds b = ctx.route->BoundsAt(f.s);
        if (f.d > b.left_m || f.d < -b.right_m) {
          violates = true;
          break;
        }
      }
    }
    terms[7] = violates ? 1.0 : 0.0;
    // consistency: mean distance to the previous choice over the first
    // seconds, at matching absolute times.
    if (ctx.previous != nullptr && !ctx.previous->empty()) {
      double sum = 0.0;
      int count = 0;
      for (const nuway_common::TrajectoryPoint& p : traj) {
        if (p.t > weights_.consistency_horizon_s) {
          break;
        }
        const nuway_common::TrajectoryPoint q =
            nuway_common::Interpolate(*ctx.previous, p.t + ctx.previous_age_s);
        sum += std::hypot(p.x - q.x, p.y - q.y);
        ++count;
      }
      if (count > 0) {
        terms[8] = std::min(1.0, sum / static_cast<double>(count) /
                                     weights_.consistency_norm_m);
      }
    }
  }
  terms[9] = c->qp_relaxed ? 1.0 : 0.0;

  const std::array<double, kNumCostTerms> w = weights_.AsArray();
  c->cost_breakdown.assign(kNumCostTerms, 0.0);
  c->cost = 0.0;
  for (std::size_t i = 0; i < kNumCostTerms; ++i) {
    c->cost_breakdown[i] = w[i] * terms[i];
    c->cost += c->cost_breakdown[i];
  }
}

int RuleSelector::Select(const std::vector<Candidate>& candidates) {
  int best = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const Candidate& c = candidates[i];
    if (!(c.refined || c.injected) || !c.feasible()) {
      continue;
    }
    if (c.cost < best_cost) {
      best_cost = c.cost;
      best = static_cast<int>(i);
    }
  }
  return best;
}

}  // namespace nuway_planning
