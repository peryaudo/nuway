#include "nuway_planning/piecewise_jerk_qp.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

namespace nuway_planning {
namespace {

// Checks the piecewise-constant-jerk kinematics between the knots.
void ExpectKinematics(const PiecewiseJerkSolution& sol, double h) {
  for (std::size_t i = 0; i + 1 < sol.x.size(); ++i) {
    const double x_next = sol.x[i] + (h * sol.dx[i]) +
                          ((h * h / 3.0) * sol.ddx[i]) +
                          ((h * h / 6.0) * sol.ddx[i + 1]);
    const double dx_next =
        sol.dx[i] + ((h / 2.0) * (sol.ddx[i] + sol.ddx[i + 1]));
    EXPECT_NEAR(sol.x[i + 1], x_next, 1e-3);
    EXPECT_NEAR(sol.dx[i + 1], dx_next, 1e-3);
  }
}

// A path-like problem: 31 knots at 1 m, tracking d = 0 inside a +-0.9 m
// band except knots 10..15, where an obstacle forces d >= 1.0.
PiecewiseJerkProblem PathProblem() {
  PiecewiseJerkProblem p;
  p.step = 1.0;
  p.n = 31;
  p.x0 = {0.5, 0.0, 0.0};
  p.x_ref.assign(31, 0.0);
  p.w_x = 1.0;
  p.w_x_end = 10.0;
  p.w_ddx = 50.0;
  p.w_dddx = 500.0;
  p.x_lower.assign(31, -0.9);
  p.x_upper.assign(31, 2.5);
  p.ddx_lower.assign(31, -0.5);
  p.ddx_upper.assign(31, 0.5);
  for (int i = 10; i <= 15; ++i) {
    p.x_lower[static_cast<std::size_t>(i)] = 1.0;
  }
  return p;
}

// A speed-like problem: 81 knots at 0.1 s from 10 m/s with a lead whose
// rear (minus the gap) sits at 20 + 4 t: the ego must slow down.
PiecewiseJerkProblem SpeedProblem() {
  PiecewiseJerkProblem p;
  p.step = 0.1;
  p.n = 81;
  p.x0 = {0.0, 10.0, 0.0};
  p.dx_ref = 10.0;
  p.w_dx = 1.0;
  p.w_ddx = 5.0;
  p.w_dddx = 10.0;
  p.x_lower.assign(81, 0.0);
  p.x_upper.resize(81);
  p.dx_lower.assign(81, 0.0);
  p.dx_upper.assign(81, 15.0);
  p.ddx_lower.assign(81, -6.0);
  p.ddx_upper.assign(81, 3.0);
  p.dddx_max = 5.0;
  for (int i = 0; i < 81; ++i) {
    p.x_upper[static_cast<std::size_t>(i)] = 20.0 + (4.0 * 0.1 * i);
  }
  return p;
}

TEST(PiecewiseJerkQpTest, PathProblemRespectsTheBoundsAndKinematics) {
  PiecewiseJerkQp qp{QpSettings{}};
  const PiecewiseJerkProblem p = PathProblem();
  const PiecewiseJerkSolution sol = qp.Solve(p);
  ASSERT_EQ(sol.outcome, QpOutcome::kSolved) << sol.status;
  ASSERT_EQ(sol.x.size(), 31U);
  EXPECT_LT(sol.max_slack, 0.01);
  EXPECT_NEAR(sol.x[0], 0.5, 1e-4);
  EXPECT_NEAR(sol.dx[0], 0.0, 1e-4);
  for (std::size_t i = 0; i < 31; ++i) {
    EXPECT_GE(sol.x[i], p.x_lower[i] - 1e-3) << i;
    EXPECT_LE(sol.x[i], p.x_upper[i] + 1e-3) << i;
    EXPECT_LE(std::abs(sol.ddx[i]), 0.5 + 1e-3) << i;
  }
  EXPECT_GE(sol.x[12], 1.0 - 1e-3);
  EXPECT_LT(std::abs(sol.x[30]), 0.3);  // back toward the reference
  ExpectKinematics(sol, 1.0);
}

TEST(PiecewiseJerkQpTest, SpeedProblemStaysBehindTheLeadAndWarmStarts) {
  PiecewiseJerkQp qp{QpSettings{}};
  const PiecewiseJerkProblem p = SpeedProblem();
  const PiecewiseJerkSolution first = qp.Solve(p);
  ASSERT_EQ(first.outcome, QpOutcome::kSolved) << first.status;
  for (std::size_t i = 0; i < 81; ++i) {
    // The quadratic slack lets a bound give by up to slack_tolerance.
    EXPECT_LE(first.x[i], p.x_upper[i] + 1e-2) << i;
    EXPECT_GE(first.dx[i], -1e-3) << i;
    EXPECT_GE(first.ddx[i], -6.0 - 1e-3) << i;
    EXPECT_LE(first.ddx[i], 3.0 + 1e-3) << i;
    if (i + 1 < 81) {
      EXPECT_LE(std::abs(first.ddx[i + 1] - first.ddx[i]), 0.5 + 1e-3) << i;
    }
  }
  EXPECT_LT(first.dx[80], 6.0);  // slowed toward the lead's 4 m/s
  ExpectKinematics(first, 0.1);
  // The same problem again: OSQP starts from the previous solution.
  const PiecewiseJerkSolution second = qp.Solve(p);
  ASSERT_EQ(second.outcome, QpOutcome::kSolved) << second.status;
  EXPECT_LT(second.iterations, first.iterations);
  EXPECT_NEAR(second.x[80], first.x[80], 1e-2);
  // Shifted by a knot, then solved from the shifted state: still solved.
  qp.ShiftWarmStart();
  PiecewiseJerkProblem next = p;
  next.x0 = {first.x[1], first.dx[1], first.ddx[1]};
  const PiecewiseJerkSolution third = qp.Solve(next);
  EXPECT_EQ(third.outcome, QpOutcome::kSolved) << third.status;
  EXPECT_LT(third.iterations, first.iterations);
}

TEST(PiecewiseJerkQpTest, ExhaustedBudgetFailsAndInfeasibleBoxRelaxes) {
  QpSettings tight;
  tight.max_iter = 1;
  tight.check_termination = 1;
  PiecewiseJerkQp starved{tight};
  const PiecewiseJerkSolution out = starved.Solve(SpeedProblem());
  EXPECT_EQ(out.outcome, QpOutcome::kFailed);
  EXPECT_EQ(out.status, "max_iter_reached");
  EXPECT_TRUE(out.x.empty());

  // A lead too close to brake for (8 m ahead at 10 m/s, 6 m/s^2 and 5
  // m/s^3 allow no such stop): the box gives through the slack, the
  // dynamics stay intact.
  PiecewiseJerkQp qp{QpSettings{}};
  PiecewiseJerkProblem p = SpeedProblem();
  for (std::size_t i = 0; i < 81; ++i) {
    p.x_upper[i] = 8.0 + (4.0 * 0.1 * static_cast<double>(i));
  }
  const PiecewiseJerkSolution relaxed = qp.Solve(p);
  ASSERT_EQ(relaxed.outcome, QpOutcome::kRelaxed) << relaxed.status;
  EXPECT_EQ(relaxed.status, "solved_relaxed");
  EXPECT_GT(relaxed.max_slack, 0.01);
  EXPECT_LT(relaxed.max_slack, 0.5);
  EXPECT_LT(relaxed.iterations, 2000);
  for (std::size_t i = 0; i < 81; ++i) {
    EXPECT_GE(relaxed.ddx[i], -6.0 - 1e-3) << i;
  }
  ExpectKinematics(relaxed, 0.1);

  // A box closed entirely (x_upper below x_lower): OSQP refuses l > u on
  // a row, so the row collapses to the midpoint and the slack carries it.
  // Metres of slack need tens of thousands of iterations at the fixed rho
  // (27670 here), which is why the refiner never submits such a box; the
  // QP only has to fail cleanly, without OSQP's update error, and keep
  // its workspace usable for the next call.
  PiecewiseJerkProblem closed = SpeedProblem();
  for (std::size_t i = 0; i < 20; ++i) {
    closed.x_upper[i] = closed.x_lower[i] - 2.0;
  }
  const PiecewiseJerkSolution collapsed = qp.Solve(closed);
  EXPECT_EQ(collapsed.outcome, QpOutcome::kFailed);
  EXPECT_EQ(collapsed.status, "max_iter_reached");
  EXPECT_EQ(qp.Solve(SpeedProblem()).outcome, QpOutcome::kSolved);

  PiecewiseJerkProblem bad;
  bad.n = 1;
  EXPECT_EQ(qp.Solve(bad).outcome, QpOutcome::kFailed);
  EXPECT_EQ(qp.Solve(bad).status, "malformed");
  EXPECT_STREQ(QpOutcomeName(QpOutcome::kRelaxed), "relaxed");
}

}  // namespace
}  // namespace nuway_planning
