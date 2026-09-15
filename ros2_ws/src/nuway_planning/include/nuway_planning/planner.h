// The planner pipeline (M1 §3.3-§3.6) as a library: sample the lattice for
// the behavior decision (or the injected pair alone without one), filter
// (kinematics, bounds, collision), score, refine the top K by the QPs,
// re-check and re-score the refined shapes, and select. planner_node is
// the ROS shell; nuway_py binds this class for the M6 expert. No rclcpp.
#ifndef NUWAY_PLANNING_PLANNER_H_
#define NUWAY_PLANNING_PLANNER_H_

#include <optional>
#include <string>
#include <vector>

#include <nuway_common/trajectory.h>

#include "nuway_planning/behavior_fsm.h"
#include "nuway_planning/candidate.h"
#include "nuway_planning/candidate_refiner.h"
#include "nuway_planning/collision_checker.h"
#include "nuway_planning/lattice_sampler.h"
#include "nuway_planning/rule_selector.h"
#include "nuway_planning/scene.h"

namespace nuway_planning {

struct PlannerOptions {
  LatticeOptions lattice;
  LatticeLimits limits;
  CollisionOptions collision;
  RefinerOptions refiner;
  SelectorWeights weights;
  int top_k = 8;               // candidates refined by the QPs (§3.5)
  int publish_max = 32;        // candidates returned for the viz
  double tick_period_s = 0.2;  // planning tick, for the consistency term
};

// One planning tick's result. `candidates` holds the refined and injected
// candidates first (sorted by cost), then the best of the remaining
// feasible normals up to publish_max, all scored; `selected_index` points
// into it, -1 for the no-input case (no route line or an ego off it).
struct PlanResult {
  std::vector<Candidate> candidates;
  int selected_index = -1;
  bool no_input = false;
  bool used_decision = false;  // false: the injected pair alone (§3.6 case 2)
  int sampled = 0;
  int feasible = 0;    // normal candidates past the filter and the check
  int refined = 0;     // by the QPs
  int qp_failed = 0;   // kept their lattice shape
  int qp_relaxed = 0;  // refined through slack
  double ego_s = 0.0;
  double ego_d = 0.0;
  // The cycle breakdown of the task 17 tuning note (milliseconds): the
  // lattice sample + filter, every collision check, the K refinements (two
  // QPs each plus the re-check), the scoring/selection/assembly.
  int path_iterations = 0;   // OSQP iterations summed over the path QPs
  int speed_iterations = 0;  // and the speed QPs
  double solve_ms = 0.0;     // wall clock inside the OSQP solves (of refine_ms)
  double sample_ms = 0.0;
  double check_ms = 0.0;
  double refine_ms = 0.0;
  double select_ms = 0.0;
  std::string message;

  const Candidate* selected() const {
    return selected_index >= 0
               ? &candidates[static_cast<std::size_t>(selected_index)]
               : nullptr;
  }
};

class Planner {
 public:
  explicit Planner(PlannerOptions options);

  // `decision` nullopt, or the no-input decision, is case 2 of §3.6. The
  // caller has already checked the pose (case 3 is its call).
  PlanResult Plan(const SceneInput& in,
                  const std::optional<BehaviorOutput>& decision);

  // Drops the cross-tick state (ResetEvent).
  void Reset();

  const PlannerOptions& options() const { return options_; }

 private:
  PlannerOptions options_;
  LatticeSampler sampler_;
  CollisionChecker checker_;
  CandidateRefiner refiner_;
  RuleSelector selector_;
  std::optional<double> s_hint_;
  std::optional<nuway_common::Trajectory> previous_;
};

}  // namespace nuway_planning

#endif  // NUWAY_PLANNING_PLANNER_H_
