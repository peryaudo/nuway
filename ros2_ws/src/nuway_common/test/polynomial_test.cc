#include "nuway_common/polynomial.h"

#include <gtest/gtest.h>

namespace nuway_common {
namespace {

TEST(PolynomialTest, QuinticMeetsBothBoundaryStates) {
  const Polynomial5 p =
      Polynomial5::Quintic(1.0, 0.5, -0.2, 4.0, 0.0, 0.0, 7.0);
  EXPECT_NEAR(p.Eval(0.0), 1.0, 1e-12);
  EXPECT_NEAR(p.EvalFirst(0.0), 0.5, 1e-12);
  EXPECT_NEAR(p.EvalSecond(0.0), -0.2, 1e-12);
  EXPECT_NEAR(p.Eval(7.0), 4.0, 1e-9);
  EXPECT_NEAR(p.EvalFirst(7.0), 0.0, 1e-9);
  EXPECT_NEAR(p.EvalSecond(7.0), 0.0, 1e-9);
}

TEST(PolynomialTest, QuarticReachesTheEndVelocityWithFreePosition) {
  const Polynomial5 p = Polynomial5::Quartic(0.0, 5.0, 0.0, 12.0, 0.0, 6.0);
  EXPECT_NEAR(p.EvalFirst(0.0), 5.0, 1e-12);
  EXPECT_NEAR(p.EvalFirst(6.0), 12.0, 1e-9);
  EXPECT_NEAR(p.EvalSecond(6.0), 0.0, 1e-9);
  EXPECT_EQ(p.coefficients()[5], 0.0);
  // Speed-up: the distance covered is between 5 * 6 and 12 * 6.
  EXPECT_GT(p.Eval(6.0), 30.0);
  EXPECT_LT(p.Eval(6.0), 72.0);
}

TEST(PolynomialTest, DerivativesAreConsistentWithFiniteDifferences) {
  const Polynomial5 p =
      Polynomial5::Quintic(0.0, 1.0, 0.5, 3.0, -1.0, 0.25, 4.0);
  const double h = 1e-4;
  for (int i = 0; i <= 8; ++i) {
    const double t = 0.5 * i;
    const double first = (p.Eval(t + h) - p.Eval(t - h)) / (2.0 * h);
    const double second = (p.EvalFirst(t + h) - p.EvalFirst(t - h)) / (2.0 * h);
    const double third =
        (p.EvalSecond(t + h) - p.EvalSecond(t - h)) / (2.0 * h);
    EXPECT_NEAR(p.EvalFirst(t), first, 1e-6);
    EXPECT_NEAR(p.EvalSecond(t), second, 1e-6);
    EXPECT_NEAR(p.EvalThird(t), third, 1e-6);
  }
}

TEST(PolynomialTest, NonPositiveLengthHoldsTheStart) {
  const Polynomial5 p = Polynomial5::Quintic(2.0, 1.0, 1.0, 9.0, 9.0, 9.0, 0.0);
  EXPECT_EQ(p.Eval(3.0), 2.0);
  EXPECT_EQ(p.EvalFirst(3.0), 0.0);
}

}  // namespace
}  // namespace nuway_common
