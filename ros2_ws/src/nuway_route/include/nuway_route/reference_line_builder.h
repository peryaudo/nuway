// ReferenceLineBuilder (M0 §2.5): turns an ordered lane list into the
// ReferenceLine message the controller and planners follow. No rclcpp.
//
// The line is built in four steps (reference_line_builder.cc teaches each):
//   1. Concatenate the lane centerlines along the route into one raw
//      polyline. Where the route changes lane, the polyline blends laterally
//      from the source lane onto the target over blend_length_m with a
//      quintic ease-in/out, so position, heading and curvature stay
//      continuous through the change.
//   2. Extend extension_m past the goal along the last lane's successors,
//      so a planner looking ahead never runs out of line at the goal.
//   3. Compute heading (central difference) and curvature (Menger, i.e. the
//      circumscribed circle of consecutive triples) on the raw samples,
//      which lie on the true lane geometry, and smooth the curvature with a
//      curvature_window-point moving average.
//   4. Resample at spacing_m by arc length, interpolating position, heading
//      and curvature and carrying lane id, drivable bounds (half the lane
//      plus its same-direction neighbours) and the speed limit from the
//      nearer raw sample.
#ifndef NUWAY_ROUTE_REFERENCE_LINE_BUILDER_H_
#define NUWAY_ROUTE_REFERENCE_LINE_BUILDER_H_

#include <cstdint>
#include <optional>
#include <vector>

#include <nuway_map/lane_graph.h>
#include <nuway_msgs/msg/reference_line.hpp>

namespace nuway_route {

struct ReferenceLineOptions {
  double spacing_m = 0.5;        // arc length between output samples
  double blend_length_m = 30.0;  // lateral blend at a lane change
  double extension_m = 50.0;     // line continued past the goal
  int curvature_window = 5;      // moving-average width, odd
};

// Builds the reference line message (header left empty) for an ordered lane
// list; nullopt when the list is empty or names unknown lanes. `start_s_m`
// is the ego's arc length along the first lane: a lateral blend that starts
// on that lane begins there (RoutePlan::start_s_m), so the line does not
// sit fully on the neighbour lane behind an ego that is already
// mid-section. Every blend is capped at the remaining section length so it
// completes before the section ends. Points are in the map frame (metres),
// headings in radians, curvature in 1/m (positive for a left turn).
std::optional<nuway_msgs::msg::ReferenceLine> BuildReferenceLine(
    const nuway_map::LaneGraph& graph,
    const std::vector<std::uint32_t>& lane_ids,
    const ReferenceLineOptions& options, double start_s_m = 0.0);

// Quintic ease 0 -> 1 with zero first and second derivatives at both ends:
// 10 t^3 - 15 t^4 + 6 t^5, with t clamped to [0, 1].
double QuinticBlend(double t);

}  // namespace nuway_route

#endif  // NUWAY_ROUTE_REFERENCE_LINE_BUILDER_H_
