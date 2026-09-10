// Rule selector (M1 §3.6): the weighted cost table every candidate is
// scored with, and the argmin over the candidates the planner may select
// (the K refined normal candidates plus the injected stop pair). Every
// term is normalised to roughly [0, 1] before its weight so the weights
// read as relative importances. No rclcpp.
#ifndef NUWAY_PLANNING_RULE_SELECTOR_H_
#define NUWAY_PLANNING_RULE_SELECTOR_H_

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

#include <nuway_common/trajectory.h>

#include "nuway_planning/candidate.h"
#include "nuway_planning/collision_checker.h"
#include "nuway_planning/route_line.h"

namespace nuway_planning {

// The cost terms in breakdown order (docs/02 §4 cost_breakdown_names).
constexpr std::array<const char*, 10> kCostTermNames = {
    "collision",   "ttc",     "proximity", "progress",    "speed_dev",
    "lateral_dev", "comfort", "rule",      "consistency", "qp_relaxed"};
constexpr std::size_t kNumCostTerms = kCostTermNames.size();

struct SelectorWeights {
  double collision = 1000.0;
  double ttc = 50.0;
  double proximity = 10.0;
  double progress = 20.0;
  double speed_dev = 10.0;
  double lateral_dev = 5.0;
  double comfort = 5.0;
  double rule = 500.0;
  double consistency = 3.0;
  double qp_relaxed = 30.0;
  // Normalisers of the terms.
  double ttc_horizon_s = 4.0;       // ttc = max(0, 1 - min_ttc / this)
  double proximity_range_m = 3.0;   // proximity = mean max(0, 1 - d / this)
  double a_lat_norm_mps2 = 4.0;     // comfort: (a_lat / this)^2
  double jerk_norm_mps3 = 5.0;      // comfort: (jerk / this)^2
  double consistency_norm_m = 2.0;  // consistency: mean distance / this
  double consistency_horizon_s = 2.0;
  double stop_line_tolerance_m = 0.5;  // rule: s_end past stop_s + this

  // The weights in kCostTermNames order.
  std::array<double, kNumCostTerms> AsArray() const {
    return {collision,   ttc,     proximity, progress,    speed_dev,
            lateral_dev, comfort, rule,      consistency, qp_relaxed};
  }
};

// What the terms compare a candidate against on this tick.
struct SelectorContext {
  const RouteLine* route = nullptr;
  double ego_s = 0.0;
  double v_limit_mps = 1.0;       // the posted limit at the ego
  double target_speed_mps = 0.0;  // the decision's target (0 without one)
  double lane_half_width_m = 1.75;
  std::optional<double> stop_s;  // a stop line the candidate must not pass
  // The previously selected trajectory and how long ago it was stamped,
  // for the consistency term; null on the first tick of an episode.
  const nuway_common::Trajectory* previous = nullptr;
  double previous_age_s = 0.0;
};

class RuleSelector {
 public:
  explicit RuleSelector(SelectorWeights weights);

  // Fills the candidate's cost and its weighted breakdown.
  void Score(const CollisionResult& collision, const SelectorContext& ctx,
             Candidate* candidate) const;

  // Index of the cheapest feasible candidate that is refined or injected
  // (§3.6: the argmin never lies among the unrefined normals), or -1.
  static int Select(const std::vector<Candidate>& candidates);

  const SelectorWeights& weights() const { return weights_; }

 private:
  SelectorWeights weights_;
};

}  // namespace nuway_planning

#endif  // NUWAY_PLANNING_RULE_SELECTOR_H_
