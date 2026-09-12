#include "nuway_planning/route_line.h"

#include <cmath>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include <nuway_common/frenet.h>
#include <nuway_common/geometry.h>

namespace nuway_planning {
namespace {

// A 200 m straight at 0.5 m spacing, lane 1 for the first half and lane 2
// after, limit 20 m/s dropping to 8 m/s at 150 m.
RouteLine Straight() {
  nuway_common::Vector2dList points;
  std::vector<std::uint32_t> lanes;
  std::vector<double> limits;
  for (int i = 0; i <= 400; ++i) {
    points.emplace_back(0.5 * i, 0.0);
    lanes.push_back(i < 200 ? 1U : 2U);
    limits.push_back(i < 300 ? 20.0 : 8.0);
  }
  return RouteLine(nuway_common::ReferenceLine::FromPoints(points), lanes,
                   limits, {2.0}, {1.5}, 150.0);
}

TEST(RouteLineTest, AttributesFollowTheSamples) {
  const RouteLine route = Straight();
  EXPECT_EQ(route.LaneIdAt(10.0), 1U);
  EXPECT_EQ(route.LaneIdAt(120.0), 2U);
  EXPECT_NEAR(route.SpeedLimitAt(10.0), 20.0, 1e-12);
  EXPECT_NEAR(route.SpeedLimitAt(149.8), 8.0, 1e-12);  // the drop applies early
  EXPECT_NEAR(route.BoundsAt(5.0).left_m, 2.0, 1e-12);
  EXPECT_NEAR(route.BoundsAt(5.0).right_m, 1.5, 1e-12);
  EXPECT_NEAR(route.goal_s(), 150.0, 1e-12);
  EXPECT_TRUE(route.IsRouteLaneAhead(2, 10.0));
  EXPECT_FALSE(route.IsRouteLaneAhead(1, 120.0));
  EXPECT_FALSE(route.IsRouteLaneAhead(0, 0.0));
}

TEST(RouteLineTest, SpeedBoundRampsDownBeforeALowerLimit) {
  const RouteLine route = Straight();
  const SpeedProfileOptions options;
  // 100 m before the drop: braking at 2 m/s^2 from 20 needs 84 m, so the
  // bound is the full limit there and a ramp closer in.
  EXPECT_NEAR(route.SpeedBoundAt(40.0, options), 20.0, 1e-9);
  // The drop is seen at the first sample carrying it (149.5 m: half a
  // sample early), so the ramp is v^2 = 8^2 + 2 * 2 * 19.5.
  const double at_130 = route.SpeedBoundAt(130.0, options);
  EXPECT_NEAR(at_130, std::sqrt((8.0 * 8.0) + (2.0 * 2.0 * 19.5)), 1e-6);
  EXPECT_NEAR(route.SpeedBoundAt(160.0, options), 8.0, 1e-9);
}

// The s of a projection, or NaN when it missed (clang-tidy cannot see
// through ASSERT_TRUE, so the tests unwrap explicitly).
double ProjectedS(const std::optional<nuway_common::FrenetPoint>& f) {
  return f.has_value() ? f->s : std::nan("");
}

TEST(RouteLineTest, CurvatureSpeedCapLooksAheadOverTheReach) {
  // A 40 m straight into a 10 m radius arc (kappa 0.1) after s = 40.
  nuway_common::Vector2dList points;
  for (int i = 0; i < 80; ++i) {
    points.emplace_back(0.5 * i, 0.0);
  }
  for (int i = 0; i <= 60; ++i) {
    const double theta = 0.5 * i / 10.0;
    points.emplace_back(40.0 + (10.0 * std::sin(theta)),
                        10.0 * (1.0 - std::cos(theta)));
  }
  const RouteLine route(nuway_common::ReferenceLine::FromPoints(points), {1U},
                        {20.0}, {1.75}, {1.75}, std::nullopt);
  EXPECT_TRUE(std::isinf(route.CurvatureSpeedCap(0.0, 20.0, 4.0)));
  // Reaching into the arc: sqrt(4 / 0.1) = 6.3 m/s (Menger on 0.5 m chords).
  EXPECT_NEAR(route.CurvatureSpeedCap(0.0, 60.0, 4.0), std::sqrt(40.0), 0.2);
  EXPECT_NEAR(route.CurvatureSpeedCap(50.0, 0.0, 4.0), std::sqrt(40.0), 0.2);
}

TEST(RouteLineTest, ProjectUsesTheHintAndFallsBackGlobally) {
  const RouteLine route = Straight();
  EXPECT_NEAR(ProjectedS(route.Project(50.2, 0.3, 5.0, 48.0, 10.0, 50.0)), 50.2,
              1e-9);
  // A hint far away: the window misses, the global search still finds it.
  EXPECT_NEAR(ProjectedS(route.Project(50.2, 0.3, 5.0, 190.0, 5.0, 5.0)), 50.2,
              1e-9);
  EXPECT_FALSE(
      route.Project(50.0, 30.0, 5.0, std::nullopt, 0.0, 0.0).has_value());
}

TEST(RouteLineTest, ProjectNearNeverLeavesTheWindow) {
  const RouteLine route = Straight();
  EXPECT_NEAR(ProjectedS(route.ProjectNear(50.2, 0.3, 5.0, 48.0, 10.0, 50.0)),
              50.2, 1e-9);
  // The same far hint: no global fallback, the point is off this leg.
  EXPECT_FALSE(route.ProjectNear(50.2, 0.3, 5.0, 190.0, 5.0, 5.0).has_value());
}

}  // namespace
}  // namespace nuway_planning
