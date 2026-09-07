#include "nuway_route/reference_line_builder.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "nuway_route/route_planner.hpp"
#include "test_map.hpp"

namespace nuway_route {
namespace {

using Ids = std::vector<std::uint32_t>;

nuway_msgs::msg::ReferenceLine Unwrap(
    const std::optional<nuway_msgs::msg::ReferenceLine>& maybe) {
  EXPECT_TRUE(maybe.has_value());
  return maybe.value_or(nuway_msgs::msg::ReferenceLine{});
}

double PointGap(const nuway_msgs::msg::ReferenceLine& line, std::size_t i) {
  return std::hypot(line.points[i].x - line.points[i - 1].x,
                    line.points[i].y - line.points[i - 1].y);
}

TEST(ReferenceLineBuilder, QuinticBlendEases) {
  EXPECT_DOUBLE_EQ(QuinticBlend(0.0), 0.0);
  EXPECT_DOUBLE_EQ(QuinticBlend(1.0), 1.0);
  EXPECT_DOUBLE_EQ(QuinticBlend(0.5), 0.5);
  EXPECT_DOUBLE_EQ(QuinticBlend(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(QuinticBlend(2.0), 1.0);
  EXPECT_LT(QuinticBlend(0.1), 0.1);  // slow start
}

TEST(ReferenceLineBuilder, CircularLaneCurvatureMatchesRadius) {
  const nuway_map::LaneGraph ring = BuildGraph(kCircleXodr);
  ReferenceLineOptions options;
  options.extension_m = 0.0;
  // Lane -1 sits 1.75 m outside the 30 m counter-clockwise reference circle:
  // radius 31.75, curvature +1/31.75 (Menger on the 0.5 m resample of a 1 m
  // polyline, 5-point smoothed; the ends taper because the end curvatures
  // are zero by construction).
  const nuway_msgs::msg::ReferenceLine line =
      Unwrap(BuildReferenceLine(ring, {Id(7, 0, -1)}, options));
  ASSERT_GT(line.points.size(), 300U);
  for (std::size_t i = 10; i + 10 < line.points.size(); ++i) {
    EXPECT_NEAR(line.curvature[i], 1.0 / 31.75, 2e-3) << "i=" << i;
    EXPECT_GT(line.left_bound[i], 0.0F);
    EXPECT_GT(line.right_bound[i], 0.0F);
    EXPECT_NEAR(line.left_bound[i], 1.75F, 1e-5F);
    EXPECT_NEAR(std::hypot(line.points[i].x, line.points[i].y), 31.75, 0.02);
  }
  // Lane 1 runs the other way around inside, at radius 28.25.
  const nuway_msgs::msg::ReferenceLine inner =
      Unwrap(BuildReferenceLine(ring, {Id(7, 0, 1)}, options));
  EXPECT_NEAR(inner.curvature[inner.curvature.size() / 2], -1.0 / 28.25, 2e-3);
  EXPECT_NEAR(std::hypot(inner.points[50].x, inner.points[50].y), 28.25, 0.02);
}

TEST(ReferenceLineBuilder, ThreeWaypointRouteIsOneContinuousLine) {
  const nuway_map::LaneGraph graph = BuildGraph(kCorridorXodr);
  std::string error;
  const RoutePlan plan =
      PlanRoute(graph, nuway_common::SE2{2.0, -1.75, 0.0},
                {{15.0, -1.75}, {35.0, -1.75}, {48.0, -1.75}},
                RoutePlannerOptions{}, &error)
          .value_or(RoutePlan{});
  ASSERT_FALSE(plan.lane_ids.empty()) << error;
  ReferenceLineOptions options;
  options.extension_m = 0.0;
  const nuway_msgs::msg::ReferenceLine line =
      Unwrap(BuildReferenceLine(graph, plan.lane_ids, options));
  // 50 m of route at 0.5 m: 101 points, uniformly spaced, y constant.
  ASSERT_EQ(line.points.size(), 101U);
  EXPECT_EQ(line.s.size(), line.points.size());
  EXPECT_EQ(line.heading.size(), line.points.size());
  EXPECT_EQ(line.curvature.size(), line.points.size());
  EXPECT_EQ(line.speed_limit.size(), line.points.size());
  EXPECT_EQ(line.lane_id.size(), line.points.size());
  EXPECT_NEAR(line.points.front().x, 0.0, 1e-9);
  EXPECT_NEAR(line.points.back().x, 50.0, 1e-9);
  for (std::size_t i = 1; i < line.points.size(); ++i) {
    EXPECT_NEAR(PointGap(line, i), 0.5, 1e-9) << "i=" << i;
    EXPECT_NEAR(line.points[i].y, -1.75, 1e-9);
    EXPECT_NEAR(line.heading[i], 0.0, 1e-9);
    EXPECT_NEAR(line.curvature[i], 0.0, 1e-9);
    EXPECT_NEAR(line.s[i], 0.5 * i, 1e-5);
  }
  // Lane ids hand over along the route; bounds are the drivable extent: on
  // road 1 lane -1 has lane -2 to its right (1.75 + 3.0) and nothing across
  // the centre line (1.75).
  EXPECT_EQ(line.lane_id.front(), Id(1, 0, -1));
  EXPECT_EQ(line.lane_id.back(), Id(2, 1, -1));
  EXPECT_NEAR(line.left_bound[10], 1.75F, 1e-5F);
  EXPECT_NEAR(line.right_bound[10], 4.75F, 1e-5F);
  EXPECT_NEAR(line.speed_limit[10], 30.0F * 0.44704F, 1e-4F);  // road 1: 30 mph
  EXPECT_NEAR(line.speed_limit[80], 8.33F, 1e-4F);  // road 2: default
}

TEST(ReferenceLineBuilder, ExtendsBeyondGoal) {
  const nuway_map::LaneGraph graph = BuildGraph(kCorridorXodr);
  ReferenceLineOptions options;
  options.extension_m = 50.0;
  // Route ends on road 1: the extension continues through the junction road
  // and road 2 (20 m in total, then a dead end).
  const nuway_msgs::msg::ReferenceLine line =
      Unwrap(BuildReferenceLine(graph, {Id(1, 0, -1)}, options));
  EXPECT_NEAR(line.points.back().x, 50.0, 1e-9);
  EXPECT_EQ(line.lane_id.back(), Id(2, 1, -1));
  // On the ring the extension wraps around by 50 m.
  const nuway_map::LaneGraph ring = BuildGraph(kCircleXodr);
  const nuway_msgs::msg::ReferenceLine loop =
      Unwrap(BuildReferenceLine(ring, {Id(7, 0, -1)}, options));
  const double lane_length = 2.0 * 3.14159265358979323846 * 31.75;
  EXPECT_NEAR(loop.s.back(), 2.0 * lane_length, 0.6);
}

TEST(ReferenceLineBuilder, LaneChangeBlendsLaterally) {
  const nuway_map::LaneGraph graph = BuildGraph(kCorridorXodr);
  ReferenceLineOptions options;
  options.extension_m = 0.0;
  options.blend_length_m = 20.0;
  // -1 -> -2 on road 1, then straight on: y eases from -1.75 to -5.0 over
  // the first 20 m and stays there.
  const nuway_msgs::msg::ReferenceLine line = Unwrap(BuildReferenceLine(
      graph, {Id(1, 0, -1), Id(1, 0, -2), Id(5, 0, -2), Id(2, 0, -2)},
      options));
  EXPECT_NEAR(line.points.front().y, -1.75, 1e-9);
  // Halfway through the blend (x = 10) the offset is half done.
  for (std::size_t i = 0; i < line.points.size(); ++i) {
    if (std::abs(line.points[i].x - 10.0) < 0.3) {
      EXPECT_NEAR(line.points[i].y, -1.75 + (0.5 * -3.25), 0.1) << "i=" << i;
    }
  }
  for (std::size_t i = 0; i < line.points.size(); ++i) {
    if (line.points[i].x > 20.5) {
      EXPECT_NEAR(line.points[i].y, -5.0, 1e-6) << "i=" << i;
    }
    EXPECT_GT(line.left_bound[i], 0.0F);
    EXPECT_GT(line.right_bound[i], 0.0F);
  }
  // The line ends on the last lane's end point even though the blend made
  // its length a non-multiple of the spacing.
  EXPECT_NEAR(line.points.back().x, 45.0, 1e-9);
  EXPECT_LT(PointGap(line, line.points.size() - 1), 0.5 + 1e-9);
  // Empty or unknown lanes fail.
  EXPECT_FALSE(BuildReferenceLine(graph, {}, options).has_value());
  EXPECT_FALSE(BuildReferenceLine(graph, {12345U}, options).has_value());
}

}  // namespace
}  // namespace nuway_route
