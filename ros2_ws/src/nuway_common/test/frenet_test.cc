#include "nuway_common/frenet.h"

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

// Leg A runs along +x, leg B comes back and crosses it at (40, 0) heading
// -y. A point just left of the crossing is nearer to leg B, so the global
// projection lands there; a follower on leg A (hint s = 40) must not.
ReferenceLine SelfCrossing() {
  Vector2dList points;
  for (int i = 0; i <= 160; ++i) {
    points.emplace_back(0.5 * i, 0.0);  // A: (0,0) -> (80,0)
  }
  for (int i = 1; i <= 40; ++i) {
    points.emplace_back(80.0, 0.5 * i);  // up to (80,20)
  }
  for (int i = 1; i <= 80; ++i) {
    points.emplace_back(80.0 - (0.5 * i), 20.0);  // back to (40,20)
  }
  for (int i = 1; i <= 80; ++i) {
    points.emplace_back(40.0, 20.0 - (0.5 * i));  // B: down through (40,0)
  }
  return ReferenceLine::FromPoints(points);
}

TEST(FrenetTest, WindowedProjectionStaysOnTheHintedLeg) {
  const ReferenceLine line = SelfCrossing();
  // Global: leg B (s = 80 + 20 + 40 + 20 = 160 at the crossing), d = +0.15
  // (+x is to the left of a -y heading).
  const FrenetPoint global = Unwrap(line.ToFrenet(40.15, 0.3));
  EXPECT_NEAR(global.s, 160.0 - 0.3, 1e-6);
  EXPECT_NEAR(global.d, 0.15, 1e-6);
  // Windowed around leg A: s = 40.15, d = +0.3.
  const FrenetPoint near =
      Unwrap(line.ToFrenetNear(40.15, 0.3, 5.0, 40.0, 10.0, 50.0));
  EXPECT_NEAR(near.s, 40.15, 1e-6);
  EXPECT_NEAR(near.d, 0.3, 1e-6);
  // Nothing of leg A within max_dist of a point far off it: nullopt, and the
  // caller falls back to the global projection.
  EXPECT_FALSE(
      line.ToFrenetNear(40.0, 15.0, 5.0, 40.0, 10.0, 50.0).has_value());
  // The window is clamped to the line.
  EXPECT_TRUE(line.ToFrenetNear(1.0, 0.2, 5.0, -100.0, 10.0, 50.0).has_value());
}

}  // namespace
}  // namespace nuway_common

namespace nuway_common {
namespace {

// A full circle of radius r sampled every `step` m (kappa_r constant, so
// the state conversion has no curvature-rate term and round-trips exactly
// up to the polyline's own curvature error).
ReferenceLine MakeCircle(double radius, double step) {
  Vector2dList points;
  const double total = 2.0 * kPi * radius;
  const int n = static_cast<int>(std::floor(total / step)) + 1;
  for (int i = 0; i < n; ++i) {
    const double theta = (i * step) / radius;
    points.emplace_back(radius * std::cos(theta), radius * std::sin(theta));
  }
  return ReferenceLine::FromPoints(points);
}

// See Unwrap above: ASSERT_TRUE is opaque to the optional-access check.
FrenetState UnwrapState(const std::optional<FrenetState>& maybe) {
  EXPECT_TRUE(maybe.has_value());
  return maybe.value_or(FrenetState{});
}

TEST(FrenetTest, StateRoundTripOnAnArc) {
  const ReferenceLine line = MakeCircle(40.0, 0.5);
  for (int i = 0; i < 40; ++i) {
    FrenetState f;
    f.s = 5.0 + (i * 4.0);
    f.s_dot = 8.0 + (0.1 * i);
    f.s_ddot = -0.5 + (0.05 * i);
    f.d = -2.0 + (0.1 * i);
    f.d_prime = 0.05 * ((i % 5) - 2);
    f.d_dprime = 0.01 * ((i % 3) - 1);
    const CartesianState c = line.ToCartesianState(f);
    const FrenetState back = UnwrapState(line.ToFrenetState(c));
    // The polyline curvature is Menger on 0.5 m chords (1e-4 relative), so
    // the derivative terms round-trip to ~1e-5.
    EXPECT_NEAR(back.s, f.s, 1e-6);
    EXPECT_NEAR(back.d, f.d, 1e-6);
    EXPECT_NEAR(back.s_dot, f.s_dot, 1e-6);
    EXPECT_NEAR(back.d_prime, f.d_prime, 1e-6);
    EXPECT_NEAR(back.s_ddot, f.s_ddot, 1e-4);
    EXPECT_NEAR(back.d_dprime, f.d_dprime, 1e-4);
  }
}

TEST(FrenetTest, StateOnTheLineHasTheLineHeadingAndCurvature) {
  const ReferenceLine line = MakeCircle(40.0, 0.5);
  FrenetState f;
  f.s = 30.0;
  f.s_dot = 10.0;
  f.s_ddot = 1.0;
  const CartesianState c = line.ToCartesianState(f);
  EXPECT_NEAR(c.yaw, line.HeadingAt(30.0), 1e-9);
  EXPECT_NEAR(c.v, 10.0, 1e-9);
  EXPECT_NEAR(c.a, 1.0, 1e-9);
  EXPECT_NEAR(c.kappa, 1.0 / 40.0, 1e-4);
  // Left of a left bend the same s_dot is a slower ground speed: the arc
  // radius shrinks to 40 - d.
  f.d = 2.0;
  const CartesianState inner = line.ToCartesianState(f);
  EXPECT_NEAR(inner.v, 10.0 * (1.0 - (2.0 / 40.0)), 1e-3);
  EXPECT_NEAR(inner.kappa, 1.0 / 38.0, 1e-4);
}

TEST(FrenetTest, VelocityProjectionOnAStraightLine) {
  Vector2dList points;
  for (int i = 0; i <= 100; ++i) {
    points.emplace_back(0.5 * i, 0.0);
  }
  const ReferenceLine line = ReferenceLine::FromPoints(points);
  // Heading 30 deg off the line at 10 m/s: s_dot = v cos, d' = tan.
  CartesianState c;
  c.x = 20.0;
  c.y = 1.0;
  c.yaw = kPi / 6.0;
  c.v = 10.0;
  c.a = 0.0;
  c.kappa = 0.0;
  const FrenetState f = UnwrapState(line.ToFrenetState(c));
  EXPECT_NEAR(f.s_dot, 10.0 * std::cos(kPi / 6.0), 1e-9);
  EXPECT_NEAR(f.d_prime, std::tan(kPi / 6.0), 1e-9);
  EXPECT_NEAR(f.d_dprime, 0.0, 1e-9);
  EXPECT_NEAR(f.s_ddot, 0.0, 1e-9);
  const FrenetState near =
      UnwrapState(line.ToFrenetStateNear(c, 5.0, 18.0, 5.0, 10.0));
  EXPECT_NEAR(near.s, 20.0, 1e-9);
}

}  // namespace
}  // namespace nuway_common
