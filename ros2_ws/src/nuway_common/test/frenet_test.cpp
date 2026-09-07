#include "nuway_common/frenet.hpp"

#include <cmath>
#include <optional>

#include <gtest/gtest.h>

namespace nuway_common {
namespace {

// A quarter circle of radius r centred at the origin, sampled every `step` m.
ReferenceLine MakeArc(double radius, double step) {
  Vector2dList points;
  const double total = kPi / 2.0 * radius;
  const int n = static_cast<int>(std::floor(total / step)) + 1;
  for (int i = 0; i < n; ++i) {
    const double theta = (i * step) / radius;
    points.emplace_back(radius * std::cos(theta), radius * std::sin(theta));
  }
  return ReferenceLine::FromPoints(points);
}

// gtest's ASSERT macros are opaque to bugprone-unchecked-optional-access, so
// tests unwrap through this helper.
FrenetPoint Unwrap(const std::optional<FrenetPoint>& maybe) {
  EXPECT_TRUE(maybe.has_value());
  return maybe.value_or(FrenetPoint{-1.0, -1.0});
}

TEST(FrenetTest, RoundTripOnArcWithinTolerance) {
  const ReferenceLine line = MakeArc(30.0, 0.5);
  // Interior points at lateral offsets well inside 1/curvature: the round trip
  // cartesian -> frenet -> cartesian must be exact to 1e-6 (M0 task 4).
  for (int si = 0; si < 120; ++si) {
    const double s = 1.0 + (si * 0.37);
    for (int di = -4; di <= 4; ++di) {
      const double d = di * 0.5;
      const CartesianPoint cart = line.ToCartesian(FrenetPoint{s, d});
      const FrenetPoint back = Unwrap(line.ToFrenet(cart.x, cart.y));
      EXPECT_NEAR(back.s, s, 1e-6);
      EXPECT_NEAR(back.d, d, 1e-6);
      const CartesianPoint again = line.ToCartesian(back);
      EXPECT_NEAR(again.x, cart.x, 1e-6);
      EXPECT_NEAR(again.y, cart.y, 1e-6);
    }
  }
}

TEST(FrenetTest, CurvatureOfCircleMatchesInverseRadius) {
  const ReferenceLine line = MakeArc(20.0, 0.5);
  // Menger curvature of a 0.5 m chord on a 20 m circle is exact to ~1e-4.
  for (int si = 2; si < 28; ++si) {
    EXPECT_NEAR(line.CurvatureAt(static_cast<double>(si)), 1.0 / 20.0, 1e-4);
  }
}

TEST(FrenetTest, LeftOfLineIsPositive) {
  Vector2dList points;
  for (int i = 0; i <= 10; ++i) {
    points.emplace_back(static_cast<double>(i), 0.0);
  }
  const ReferenceLine line = ReferenceLine::FromPoints(points);
  const FrenetPoint frenet = Unwrap(line.ToFrenet(4.25, 1.5));
  EXPECT_NEAR(frenet.s, 4.25, 1e-9);
  EXPECT_NEAR(frenet.d, 1.5, 1e-9);
  EXPECT_NEAR(line.HeadingAt(4.25), 0.0, 1e-9);
  EXPECT_NEAR(line.length(), 10.0, 1e-9);
}

TEST(FrenetTest, FarPointBeyondMaxDistIsRejected) {
  Vector2dList points;
  points.emplace_back(0.0, 0.0);
  points.emplace_back(10.0, 0.0);
  const ReferenceLine line = ReferenceLine::FromPoints(points);
  EXPECT_FALSE(line.ToFrenet(5.0, 50.0, 10.0).has_value());
  EXPECT_TRUE(line.ToFrenet(5.0, 5.0, 10.0).has_value());
}

}  // namespace
}  // namespace nuway_common
