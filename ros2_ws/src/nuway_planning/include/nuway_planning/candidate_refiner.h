// Candidate refiner (M1 §3.5): Apollo-style path-speed decomposition on
// one lattice candidate. The path QP re-optimises d(s) at 1 m knots around
// the lattice path inside the drivable bounds, tightened by static or slow
// agents on the side the lattice path passes them, with |d''| bounded by
// the curvature budget kappa_phys - |kappa_ref|. The speed QP then
// re-optimises s(t) at 0.1 s knots along the refined path against the
// speed limit, the curvature speed, the acceleration and jerk limits, and
// the S-T boxes of the dynamic agents' prediction samples projected onto
// the path, keeping the lattice profile's side of every box (behind it:
// an upper bound on s; ahead of it: a lower bound). Either QP failing (no
// converged iterate) keeps the candidate's lattice shape with qp_relaxed
// set; a slack-relaxed solve keeps the refined shape with qp_relaxed set.
// No rclcpp.
#ifndef NUWAY_PLANNING_CANDIDATE_REFINER_H_
#define NUWAY_PLANNING_CANDIDATE_REFINER_H_

#include <string>

#include <nuway_common/frenet.h>

#include "nuway_planning/candidate.h"
#include "nuway_planning/collision_checker.h"
#include "nuway_planning/lattice_sampler.h"
#include "nuway_planning/piecewise_jerk_qp.h"
#include "nuway_planning/scene.h"

namespace nuway_planning {

struct RefinerOptions {
  double path_ds_m = 1.0;
  double w_d = 1.0;               // path: tracking of the lattice d
  double w_dd = 50.0;             // path: d'' (curvature) penalty
  double w_ddd = 500.0;           // path: d''' penalty
  double w_end = 10.0;            // path: end offset tracking
  double w_v = 1.0;               // speed: tracking of the target speed
  double w_a = 5.0;               // speed: acceleration penalty
  double w_j = 10.0;              // speed: jerk penalty
  double w_s = 0.1;               // speed: tracking of the lattice s(t)
  double static_speed_mps = 0.5;  // slower agents bound the path laterally
  double jerk_max_mps3 = 5.0;
  double min_curvature_budget = 0.02;  // floor of kappa_phys - |kappa_ref|
  QpSettings qp;
};

// One refinement's outcome, for the planner's diagnostics.
struct RefineOutcome {
  QpOutcome path = QpOutcome::kFailed;
  QpOutcome speed = QpOutcome::kFailed;
  bool path_skipped = false;  // too short a candidate for a path QP
  int path_iterations = 0;    // OSQP iterations (the task 17 tuning note)
  int speed_iterations = 0;
  std::string message;

  bool refined() const { return speed != QpOutcome::kFailed; }
};

class CandidateRefiner {
 public:
  CandidateRefiner(RefinerOptions options, LatticeLimits limits,
                   CollisionOptions collision);

  // Refines the candidate in place (trajectory, frenet, refined,
  // qp_relaxed). `ego` is the Frenet state the candidate started from.
  RefineOutcome Refine(const SceneInput& in,
                       const nuway_common::FrenetState& ego,
                       Candidate* candidate);

  // Advances the speed QP's warm start by one planning tick (two 0.1 s
  // knots); call once per planning tick before refining.
  void OnPlanningTick();

  // Drops the solver workspaces (ResetEvent).
  void Reset();

  const RefinerOptions& options() const { return options_; }

 private:
  RefinerOptions options_;
  LatticeLimits limits_;
  CollisionOptions collision_;
  PiecewiseJerkQp path_qp_;
  PiecewiseJerkQp speed_qp_;
};

}  // namespace nuway_planning

#endif  // NUWAY_PLANNING_CANDIDATE_REFINER_H_
