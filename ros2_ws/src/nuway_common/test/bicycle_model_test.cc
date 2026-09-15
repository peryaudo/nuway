#include "nuway_common/bicycle_model.h"

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

namespace nuway_common {
namespace {

constexpr double kDt = 0.05;

BicycleParams Lincoln() {
  BicycleParams p;
  p.wheelbase_m = 2.86;
  p.tau_steer_s = 0.05;
  p.accel_offset_mps2 = 0.3;
  return p;
}

// A handful of operating points covering both signs of everything.
BicycleState StateAt(int i) {
  BicycleState x;
  x << 3.0 * i, -1.5 * i, (0.7 * i) - 1.0, 2.0 + (3.0 * i), (0.15 * i) - 0.3;
  return x;
}

BicycleInput InputAt(int i) {
  BicycleInput u;
  u << 1.0 - (0.8 * i), (0.1 * i) - 0.2;
  return u;
}

TEST(BicycleModelTest, DerivativeMatchesTheEquations) {
  const BicycleParams p = Lincoln();
  BicycleState x;
  x << 1.0, 2.0, 0.5, 10.0, 0.1;
  BicycleInput u;
  u << 0.7, 0.3;
  const BicycleState dx = BicycleDerivative(x, u, p);
  EXPECT_NEAR(dx[kBicycleX], 10.0 * std::cos(0.5), 1e-12);
  EXPECT_NEAR(dx[kBicycleY], 10.0 * std::sin(0.5), 1e-12);
  EXPECT_NEAR(dx[kBicyclePsi], 10.0 * std::tan(0.1) / 2.86, 1e-12);
  EXPECT_NEAR(dx[kBicycleV], 0.7 - 0.3, 1e-12);
  EXPECT_NEAR(dx[kBicycleDelta], (0.3 - 0.1) / 0.05, 1e-12);
}

TEST(BicycleModelTest, Rk2StepIsSecondOrderAccurate) {
  // Against a fine Euler integration of the same equations: the RK2 error
  // over one 0.05 s step is O(dt^3), the Euler reference's O(h) at h = 1e-5.
  // A lag of 0.2 s keeps the step non-stiff (dt / tau = 0.25); at the
  // Lincoln's 0.05 s the midpoint rule is stable but keeps 50 % of the
  // steer error per step instead of exp(-1) = 37 %, see the header.
  BicycleParams p = Lincoln();
  p.tau_steer_s = 0.2;
  BicycleState x;
  x << 0.0, 0.0, 0.3, 8.0, 0.05;
  BicycleInput u;
  u << 1.5, 0.2;
  const BicycleState rk2 = BicycleStepRk2(x, u, kDt, p);
  BicycleState fine = x;
  const int n = 5000;
  const double h = kDt / n;
  for (int i = 0; i < n; ++i) {
    fine += h * BicycleDerivative(fine, u, p);
  }
  // The midpoint rule's error (dt^3 times the third derivatives, here
  // dominated by the yaw rate's response to the steer step) is a few 1e-4;
  // a single Euler step of the same length is an order of magnitude worse.
  const BicycleState euler = x + (kDt * BicycleDerivative(x, u, p));
  for (int k = 0; k < kBicycleStateDim; ++k) {
    EXPECT_NEAR(rk2[k], fine[k], 1e-3) << "state " << k;
    EXPECT_LT(std::abs(rk2[k] - fine[k]),
              std::max(1e-6, std::abs(euler[k] - fine[k]) / 5.0))
        << "state " << k;
  }
}

TEST(BicycleModelTest, JacobiansMatchCentralDifferences) {
  const BicycleParams p = Lincoln();
  const double eps = 1e-6;
  for (int i = 0; i < 5; ++i) {
    const BicycleState x = StateAt(i);
    const BicycleInput u = InputAt(i);
    const BicycleLinearization lin = BicycleLinearizeRk2(x, u, kDt, p);
    EXPECT_TRUE(lin.x_next.isApprox(BicycleStepRk2(x, u, kDt, p), 1e-12));
    for (int c = 0; c < kBicycleStateDim; ++c) {
      BicycleState plus = x;
      BicycleState minus = x;
      plus[c] += eps;
      minus[c] -= eps;
      const BicycleState column =
          (BicycleStepRk2(plus, u, kDt, p) - BicycleStepRk2(minus, u, kDt, p)) /
          (2.0 * eps);
      for (int r = 0; r < kBicycleStateDim; ++r) {
        EXPECT_NEAR(lin.a(r, c), column[r], 1e-7)
            << "point " << i << " A(" << r << "," << c << ")";
      }
    }
    for (int c = 0; c < kBicycleInputDim; ++c) {
      BicycleInput plus = u;
      BicycleInput minus = u;
      plus[c] += eps;
      minus[c] -= eps;
      const BicycleState column =
          (BicycleStepRk2(x, plus, kDt, p) - BicycleStepRk2(x, minus, kDt, p)) /
          (2.0 * eps);
      for (int r = 0; r < kBicycleStateDim; ++r) {
        EXPECT_NEAR(lin.b(r, c), column[r], 1e-7)
            << "point " << i << " B(" << r << "," << c << ")";
      }
    }
    // The continuous Jacobians too.
    const BicycleStateJacobian j = BicycleDfDx(x, p);
    for (int c = 0; c < kBicycleStateDim; ++c) {
      BicycleState plus = x;
      BicycleState minus = x;
      plus[c] += eps;
      minus[c] -= eps;
      const BicycleState column =
          (BicycleDerivative(plus, u, p) - BicycleDerivative(minus, u, p)) /
          (2.0 * eps);
      for (int r = 0; r < kBicycleStateDim; ++r) {
        EXPECT_NEAR(j(r, c), column[r], 1e-6);
      }
    }
  }
}

}  // namespace
}  // namespace nuway_common
