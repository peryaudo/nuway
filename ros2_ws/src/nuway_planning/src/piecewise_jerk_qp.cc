#include "nuway_planning/piecewise_jerk_qp.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <OsqpEigen/OsqpEigen.h>

namespace nuway_planning {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// OSQP takes +-OSQP_INFTY as "no bound"; anything beyond is clipped.
double Bound(double value) {
  return std::clamp(value, -OSQP_INFTY, OSQP_INFTY);
}

const char* StatusName(OsqpEigen::Status status) {
  switch (status) {
    case OsqpEigen::Status::Solved:
      return "solved";
    case OsqpEigen::Status::SolvedInaccurate:
      return "solved_inaccurate";
    case OsqpEigen::Status::MaxIterReached:
      return "max_iter_reached";
    case OsqpEigen::Status::PrimalInfeasible:
      return "primal_infeasible";
    case OsqpEigen::Status::PrimalInfeasibleInaccurate:
      return "primal_infeasible_inaccurate";
    case OsqpEigen::Status::DualInfeasible:
      return "dual_infeasible";
    case OsqpEigen::Status::DualInfeasibleInaccurate:
      return "dual_infeasible_inaccurate";
    case OsqpEigen::Status::NonCvx:
      return "non_convex";
    case OsqpEigen::Status::Sigint:
      return "sigint";
    case OsqpEigen::Status::Unsolved:
      return "unsolved";
    default:
      return "unknown";
  }
}

bool Converged(OsqpEigen::Status status) {
  return status == OsqpEigen::Status::Solved ||
         status == OsqpEigen::Status::SolvedInaccurate;
}

// The parts of a problem that fix the matrices P and A; a change rebuilds
// the workspace, anything else is an updateGradient / updateBounds.
struct Structure {
  int n = 0;
  double step = 0.0;
  double w_x = 0.0;
  double w_x_end = 0.0;
  double w_dx = 0.0;
  double w_ddx = 0.0;
  double w_dddx = 0.0;
  bool has_x_ref = false;
  bool jerk_rows = false;
  double slack_weight = 0.0;

  bool operator==(const Structure& o) const {
    return n == o.n && step == o.step && w_x == o.w_x && w_x_end == o.w_x_end &&
           w_dx == o.w_dx && w_ddx == o.w_ddx && w_dddx == o.w_dddx &&
           has_x_ref == o.has_x_ref && jerk_rows == o.jerk_rows &&
           slack_weight == o.slack_weight;
  }
};

}  // namespace

const char* QpOutcomeName(QpOutcome outcome) {
  switch (outcome) {
    case QpOutcome::kSolved:
      return "solved";
    case QpOutcome::kRelaxed:
      return "relaxed";
    case QpOutcome::kFailed:
      return "failed";
  }
  return "unknown";
}

// Variable layout: z = [x_0..x_{n-1}, x'_0.., x''_0.., sm_0.., sp_0..],
// 5n entries; sm_i, sp_i >= 0 relax the x box of knot i downward and
// upward. Constraint rows:
//   kinematics        2 (n - 1)   equalities
//   initial state     3           equalities
//   x + sm - sp       n           [lo, hi]
//   x' box            n
//   x'' box           n
//   jerk box          n - 1       (x''_{i+1} - x''_i) / h, only with a
//                                 finite bound (an infinite row would be
//                                 scaled by OSQP into a finite huge one)
//   sm, sp            2n          [0, inf)
struct PiecewiseJerkQp::Impl {
  // One OSQP workspace per problem structure.
  struct Workspace {
    Structure structure;
    std::unique_ptr<OsqpEigen::Solver> solver;
    Eigen::VectorXd last_solution;
    bool have_solution = false;
  };
  static constexpr std::size_t kMaxWorkspaces = 4;

  QpSettings settings;
  std::vector<std::unique_ptr<Workspace>> workspaces;  // most recent last
  Eigen::VectorXd gradient;
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;

  static int Vars(int n) { return 5 * n; }
  static int Rows(int n, bool jerk_rows) {
    return (2 * (n - 1)) + 3 + n + n + n + (jerk_rows ? n - 1 : 0) + (2 * n);
  }

  // Builds P (full symmetric; OSQP keeps the upper triangle) and A.
  void BuildMatrices(const PiecewiseJerkProblem& p,
                     Eigen::SparseMatrix<double>* hessian,
                     Eigen::SparseMatrix<double>* constraints) const {
    const int n = p.n;
    const double h = p.step;
    const int ix = 0;
    const int idx = n;
    const int iddx = 2 * n;
    const int ism = 3 * n;
    const int isp = 4 * n;
    const bool jerk_rows = std::isfinite(p.dddx_max);
    std::vector<Eigen::Triplet<double>> hess;
    // 1/2 z^T P z with P = 2 H for the quadratic form H.
    for (int i = 0; i < n; ++i) {
      double wx = p.x_ref.empty() ? 0.0 : p.w_x;
      if (i == n - 1) {
        wx += p.w_x_end;
      }
      if (wx > 0.0) {
        hess.emplace_back(ix + i, ix + i, 2.0 * wx);
      }
      if (p.w_dx > 0.0) {
        hess.emplace_back(idx + i, idx + i, 2.0 * p.w_dx);
      }
      double wdd = p.w_ddx;
      // Jerk term: w (x''_{i+1} - x''_i)^2 / h^2 over the n - 1 intervals.
      const double wj = p.w_dddx / (h * h);
      if (i > 0) {
        wdd += wj;
      }
      if (i < n - 1) {
        wdd += wj;
        if (wj > 0.0) {
          hess.emplace_back(iddx + i, iddx + i + 1, -2.0 * wj);
          hess.emplace_back(iddx + i + 1, iddx + i, -2.0 * wj);
        }
      }
      if (wdd > 0.0) {
        hess.emplace_back(iddx + i, iddx + i, 2.0 * wdd);
      }
      hess.emplace_back(ism + i, ism + i, 2.0 * settings.slack_weight);
      hess.emplace_back(isp + i, isp + i, 2.0 * settings.slack_weight);
    }
    hessian->resize(Vars(n), Vars(n));
    hessian->setFromTriplets(hess.begin(), hess.end());

    std::vector<Eigen::Triplet<double>> rows;
    int r = 0;
    for (int i = 0; i + 1 < n; ++i) {
      rows.emplace_back(r, ix + i + 1, 1.0);
      rows.emplace_back(r, ix + i, -1.0);
      rows.emplace_back(r, idx + i, -h);
      rows.emplace_back(r, iddx + i, -h * h / 3.0);
      rows.emplace_back(r, iddx + i + 1, -h * h / 6.0);
      ++r;
      rows.emplace_back(r, idx + i + 1, 1.0);
      rows.emplace_back(r, idx + i, -1.0);
      rows.emplace_back(r, iddx + i, -h / 2.0);
      rows.emplace_back(r, iddx + i + 1, -h / 2.0);
      ++r;
    }
    rows.emplace_back(r++, ix, 1.0);
    rows.emplace_back(r++, idx, 1.0);
    rows.emplace_back(r++, iddx, 1.0);
    for (int i = 0; i < n; ++i) {
      rows.emplace_back(r, ix + i, 1.0);
      rows.emplace_back(r, ism + i, 1.0);
      rows.emplace_back(r, isp + i, -1.0);
      ++r;
    }
    for (int i = 0; i < n; ++i) {
      rows.emplace_back(r++, idx + i, 1.0);
    }
    for (int i = 0; i < n; ++i) {
      rows.emplace_back(r++, iddx + i, 1.0);
    }
    if (jerk_rows) {
      for (int i = 0; i + 1 < n; ++i) {
        rows.emplace_back(r, iddx + i + 1, 1.0);
        rows.emplace_back(r, iddx + i, -1.0);
        ++r;
      }
    }
    for (int i = 0; i < n; ++i) {
      rows.emplace_back(r++, ism + i, 1.0);
    }
    for (int i = 0; i < n; ++i) {
      rows.emplace_back(r++, isp + i, 1.0);
    }
    constraints->resize(Rows(n, jerk_rows), Vars(n));
    constraints->setFromTriplets(rows.begin(), rows.end());
  }

  // Fills q, l and u. The slack is always free (sigma >= 0): a feasible
  // box leaves it at ~0 under the weight, an infeasible one is violated by
  // the least-cost amount, which the outcome reports (see Solve()).
  void BuildVectors(const PiecewiseJerkProblem& p) {
    const int n = p.n;
    const double h = p.step;
    gradient = Eigen::VectorXd::Zero(Vars(n));
    for (int i = 0; i < n; ++i) {
      double wx = p.x_ref.empty() ? 0.0 : p.w_x;
      if (i == n - 1) {
        wx += p.w_x_end;
      }
      if (!p.x_ref.empty()) {
        gradient[i] = -2.0 * wx * p.x_ref[static_cast<std::size_t>(i)];
      }
      gradient[n + i] = -2.0 * p.w_dx * p.dx_ref;
    }
    const bool jerk_rows = std::isfinite(p.dddx_max);
    lower = Eigen::VectorXd::Zero(Rows(n, jerk_rows));
    upper = Eigen::VectorXd::Zero(Rows(n, jerk_rows));
    int r = 2 * (n - 1);  // kinematics rows stay [0, 0]
    for (const double v : p.x0) {
      lower[r] = v;
      upper[r] = v;
      ++r;
    }
    const auto at = [](const std::vector<double>& v, int i, double none) {
      return v.empty() ? none : v[static_cast<std::size_t>(i)];
    };
    for (int i = 0; i < n; ++i) {
      double lo = Bound(at(p.x_lower, i, -kInf));
      double hi = Bound(at(p.x_upper, i, kInf));
      if (lo > hi) {
        // A box that closed entirely (a lead already inside the ego's
        // margin): OSQP rejects l > u on a row outright, so the row
        // becomes the box's midpoint and the slack, which enters this row
        // on both sides, carries the violation like any other.
        lo = hi = 0.5 * (lo + hi);
      }
      lower[r] = lo;
      upper[r] = hi;
      ++r;
    }
    for (int i = 0; i < n; ++i) {
      lower[r] = Bound(at(p.dx_lower, i, -kInf));
      upper[r] = Bound(at(p.dx_upper, i, kInf));
      ++r;
    }
    for (int i = 0; i < n; ++i) {
      lower[r] = Bound(at(p.ddx_lower, i, -kInf));
      upper[r] = Bound(at(p.ddx_upper, i, kInf));
      ++r;
    }
    if (jerk_rows) {
      for (int i = 0; i + 1 < n; ++i) {
        lower[r] = -p.dddx_max * h;
        upper[r] = p.dddx_max * h;
        ++r;
      }
    }
    for (int i = 0; i < 2 * n; ++i) {
      lower[r] = 0.0;
      upper[r] = kInf;
      ++r;
    }
    for (int i = 0; i < Rows(n, jerk_rows); ++i) {
      lower[i] = Bound(lower[i]);
      upper[i] = Bound(upper[i]);
    }
  }

  // Creates the workspace for a new structure.
  bool Setup(const PiecewiseJerkProblem& p, Workspace* ws) {
    std::unique_ptr<OsqpEigen::Solver>& solver = ws->solver;
    solver = std::make_unique<OsqpEigen::Solver>();
    solver->settings()->setVerbosity(false);
    solver->settings()->setWarmStart(settings.warm_start);
    solver->settings()->setAdaptiveRho(settings.adaptive_rho_interval > 0);
    solver->settings()->setAdaptiveRhoInterval(
        std::max(1, settings.adaptive_rho_interval));
    solver->settings()->setRho(settings.rho);
    solver->settings()->setSigma(settings.sigma);
    solver->settings()->setScaling(settings.scaling);
    solver->settings()->setMaxIteration(settings.max_iter);
    solver->settings()->setCheckTermination(settings.check_termination);
    solver->settings()->setPolish(settings.polish);
    solver->settings()->setAbsoluteTolerance(settings.eps_abs);
    solver->settings()->setRelativeTolerance(settings.eps_rel);
    Eigen::SparseMatrix<double> hessian;
    Eigen::SparseMatrix<double> constraints;
    BuildMatrices(p, &hessian, &constraints);
    BuildVectors(p);
    solver->data()->setNumberOfVariables(Vars(p.n));
    solver->data()->setNumberOfConstraints(
        Rows(p.n, std::isfinite(p.dddx_max)));
    if (!solver->data()->setHessianMatrix(hessian) ||
        !solver->data()->setGradient(gradient) ||
        !solver->data()->setLinearConstraintsMatrix(constraints) ||
        !solver->data()->setLowerBound(lower) ||
        !solver->data()->setUpperBound(upper) || !solver->initSolver()) {
      solver.reset();
      return false;
    }
    ws->have_solution = false;
    return true;
  }

  // The workspace of `structure`, moved to the back (most recent), or a
  // fresh one set up for `p` (`created`); nullptr when the setup fails.
  Workspace* Acquire(const PiecewiseJerkProblem& p, const Structure& structure,
                     bool* created) {
    *created = false;
    for (std::size_t i = 0; i < workspaces.size(); ++i) {
      if (workspaces[i]->structure == structure) {
        std::unique_ptr<Workspace> ws = std::move(workspaces[i]);
        workspaces.erase(workspaces.begin() + static_cast<std::ptrdiff_t>(i));
        workspaces.push_back(std::move(ws));
        return workspaces.back().get();
      }
    }
    *created = true;
    auto ws = std::make_unique<Workspace>();
    ws->structure = structure;
    if (!Setup(p, ws.get())) {
      return nullptr;
    }
    if (workspaces.size() >= kMaxWorkspaces) {
      workspaces.erase(workspaces.begin());
    }
    workspaces.push_back(std::move(ws));
    return workspaces.back().get();
  }
};

PiecewiseJerkQp::PiecewiseJerkQp(QpSettings settings)
    : impl_(std::make_unique<Impl>()) {
  impl_->settings = settings;
}

PiecewiseJerkQp::~PiecewiseJerkQp() = default;
PiecewiseJerkQp::PiecewiseJerkQp(PiecewiseJerkQp&&) noexcept = default;
PiecewiseJerkQp& PiecewiseJerkQp::operator=(PiecewiseJerkQp&&) noexcept =
    default;

void PiecewiseJerkQp::Reset() { impl_->workspaces.clear(); }

void PiecewiseJerkQp::ShiftWarmStart() {
  for (const std::unique_ptr<Impl::Workspace>& ws : impl_->workspaces) {
    if (!ws->have_solution || ws->solver == nullptr) {
      continue;
    }
    const int n = ws->structure.n;
    Eigen::VectorXd shifted = ws->last_solution;
    for (int block = 0; block < 5; ++block) {
      for (int i = 0; i + 1 < n; ++i) {
        shifted[(block * n) + i] = ws->last_solution[(block * n) + i + 1];
      }
    }
    ws->solver->setPrimalVariable(shifted);
    ws->last_solution = shifted;
  }
}

PiecewiseJerkSolution PiecewiseJerkQp::Solve(const PiecewiseJerkProblem& p) {
  PiecewiseJerkSolution out;
  if (p.n < 2 || p.step <= 0.0 ||
      (!p.x_ref.empty() && p.x_ref.size() != static_cast<std::size_t>(p.n))) {
    out.status = "malformed";
    return out;
  }
  Impl& im = *impl_;
  Structure structure;
  structure.n = p.n;
  structure.step = p.step;
  structure.w_x = p.w_x;
  structure.w_x_end = p.w_x_end;
  structure.w_dx = p.w_dx;
  structure.w_ddx = p.w_ddx;
  structure.w_dddx = p.w_dddx;
  structure.has_x_ref = !p.x_ref.empty();
  structure.jerk_rows = std::isfinite(p.dddx_max);
  structure.slack_weight = im.settings.slack_weight;
  bool created = false;
  Impl::Workspace* ws = im.Acquire(p, structure, &created);
  if (ws == nullptr) {
    out.status = "setup_failed";
    return out;
  }
  if (!created) {
    // An existing workspace: update in place, keeping OSQP's iterate as
    // the warm start.
    im.BuildVectors(p);
    if (!ws->solver->updateGradient(im.gradient) ||
        !ws->solver->updateBounds(im.lower, im.upper)) {
      out.status = "update_failed";
      return out;
    }
  }
  OsqpEigen::Solver& solver = *ws->solver;
  if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
    out.status = "solver_error";
    return out;
  }
  const OsqpEigen::Status status = solver.getStatus();
  out.status = StatusName(status);
  out.iterations = static_cast<int>(solver.workspace()->info->iter);
  if (!Converged(status)) {
    out.outcome = QpOutcome::kFailed;
    return out;
  }
  const Eigen::VectorXd& z = solver.getSolution();
  if (!z.allFinite()) {
    out.status = "non_finite";
    out.outcome = QpOutcome::kFailed;
    return out;
  }
  ws->last_solution = z;
  ws->have_solution = true;
  const auto n = static_cast<Eigen::Index>(p.n);
  out.x.assign(z.data(), z.data() + n);
  out.dx.assign(z.data() + n, z.data() + (2 * n));
  out.ddx.assign(z.data() + (2 * n), z.data() + (3 * n));
  out.max_slack = z.segment(3 * n, 2 * n).maxCoeff();
  out.outcome = out.max_slack > im.settings.slack_tolerance
                    ? QpOutcome::kRelaxed
                    : QpOutcome::kSolved;
  if (out.outcome == QpOutcome::kRelaxed) {
    out.status += "_relaxed";
  }
  return out;
}

}  // namespace nuway_planning
