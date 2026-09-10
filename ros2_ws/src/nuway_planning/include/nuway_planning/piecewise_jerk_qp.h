// Piecewise-jerk QP (M1 §3.5): the one-dimensional optimisation both the
// path refinement (x = d over s) and the speed refinement (x = s over t)
// reduce to, in the form of Apollo's PiecewiseJerkProblem. The unknowns
// are the state (x, x', x'') at n uniformly spaced knots with the jerk
// x''' piecewise constant between them, so the kinematics are exact linear
// equalities:
//   x_{i+1}  = x_i + h x'_i + h^2/3 x''_i + h^2/6 x''_{i+1}
//   x'_{i+1} = x'_i + h/2 (x''_i + x''_{i+1})
// The cost tracks references on x and x', penalises x'' and the jerk, and
// the constraints are boxes on x, x', x'' and the jerk. Solved with OSQP
// through osqp-eigen with fixed settings (adaptive_rho off, fixed
// max_iter, polish on, §5) and OSQP's warm start across calls. The box on
// x carries a slack variable per knot (sigma >= 0, an L1 penalty of
// slack_weight per metre in the cost) in every solve: a feasible box
// leaves it at exactly zero, an infeasible one is violated by the
// least-cost amount and the outcome is kRelaxed (the qp_relaxed penalty
// of §3.6). The slack is always on rather
// than added after a primal-infeasible solve because OSQP's infeasibility
// certificate needs thousands of iterations at a fixed rho (§5 forbids the
// adaptive one), which would burn the budget before the relaxation ran.
// An exhausted iteration budget or a solver error is kFailed and the
// caller keeps its unrefined candidate. No rclcpp.
#ifndef NUWAY_PLANNING_PIECEWISE_JERK_QP_H_
#define NUWAY_PLANNING_PIECEWISE_JERK_QP_H_

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace nuway_planning {

enum class QpOutcome : std::uint8_t {
  kSolved,   // converged (a failed polish still counts)
  kRelaxed,  // converged with an x bound violated through its slack
  kFailed,   // iteration budget out or no usable iterate
};

const char* QpOutcomeName(QpOutcome outcome);

struct QpSettings {
  double eps_abs = 1e-4;
  double eps_rel = 1e-4;
  int max_iter = 2000;        // the fixed budget of §5
  int check_termination = 5;  // OSQP checks convergence every this many
  // ADMM step. Fixed (adaptive_rho off, §5): OSQP's automatic adaptation
  // interval is derived from the setup wall time, so it is not
  // reproducible. 1.0 was the best of the values probed on the synthetic
  // problems of piecewise_jerk_qp_test (cold: ~100 path / ~900 speed
  // iterations; warm-started: ~5). adaptive_rho_interval > 0 would switch
  // adaptation on at that fixed, iteration-counted interval, which is
  // deterministic too; it did not help the ill-conditioned relaxed cases
  // and is left off.
  double rho = 1.0;
  int adaptive_rho_interval = 0;
  double sigma = 1e-6;
  int scaling = 10;  // Ruiz equilibration iterations
  bool polish = true;
  double slack_weight = 1e4;      // relaxation weight of §3.5
  double slack_tolerance = 0.01;  // max slack above this: kRelaxed
  bool warm_start = true;
};

constexpr double kQpUnbounded = std::numeric_limits<double>::infinity();

struct PiecewiseJerkProblem {
  double step = 1.0;  // h: the knot spacing (m for the path, s for speed)
  int n = 0;          // knots, at least 2
  std::array<double, 3> x0 = {0.0, 0.0, 0.0};  // fixed initial state
  // Tracking terms (an empty x_ref disables the x term).
  std::vector<double> x_ref;
  double w_x = 0.0;
  double w_x_end = 0.0;  // extra weight on the last knot's x error
  double dx_ref = 0.0;
  double w_dx = 0.0;
  double w_ddx = 0.0;
  double w_dddx = 0.0;
  // Boxes (empty vectors mean unbounded; every non-empty one has n entries).
  std::vector<double> x_lower;
  std::vector<double> x_upper;
  std::vector<double> dx_lower;
  std::vector<double> dx_upper;
  std::vector<double> ddx_lower;
  std::vector<double> ddx_upper;
  double dddx_max = kQpUnbounded;  // |x'''| bound
};

struct PiecewiseJerkSolution {
  std::vector<double> x;
  std::vector<double> dx;
  std::vector<double> ddx;
  QpOutcome outcome = QpOutcome::kFailed;
  int iterations = 0;
  double max_slack = 0.0;  // largest x-box violation
  std::string status;      // OSQP's status name, for diagnostics
};

class PiecewiseJerkQp {
 public:
  explicit PiecewiseJerkQp(QpSettings settings);
  ~PiecewiseJerkQp();
  PiecewiseJerkQp(const PiecewiseJerkQp&) = delete;
  PiecewiseJerkQp& operator=(const PiecewiseJerkQp&) = delete;
  PiecewiseJerkQp(PiecewiseJerkQp&&) noexcept;
  PiecewiseJerkQp& operator=(PiecewiseJerkQp&&) noexcept;

  // Solves the problem. OSQP workspaces are kept per structure (n, step,
  // weights, jerk bound), a few most recently used, so a solve is
  // warm-started by the previous solution of the same structure (the K
  // candidates of a tick share two or three path lengths).
  PiecewiseJerkSolution Solve(const PiecewiseJerkProblem& problem);

  // Shifts every kept solution by one knot (x_i <- x_{i+1}, the last knot
  // repeated) before the next Solve(): the warm start of a receding
  // horizon that advanced by one step.
  void ShiftWarmStart();

  // Drops the workspace (ResetEvent).
  void Reset();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nuway_planning

#endif  // NUWAY_PLANNING_PIECEWISE_JERK_QP_H_
