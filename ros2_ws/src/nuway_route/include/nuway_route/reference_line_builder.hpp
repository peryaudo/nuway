// ReferenceLineBuilder (M0 §2.5): concatenates lane centerlines along a route,
// blends laterally over blend_length_m with a quintic at lane-change edges,
// resamples at spacing_m, computes heading and (5-point smoothed Menger)
// curvature, fills drivable bounds from lane widths plus same-direction
// neighbors, speed limits and per-point lane ids, and extends extension_m past
// the goal along the last lane's successors. No rclcpp.
#ifndef NUWAY_ROUTE_REFERENCE_LINE_BUILDER_HPP_
#define NUWAY_ROUTE_REFERENCE_LINE_BUILDER_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include <nuway_map/lane_graph.hpp>
#include <nuway_msgs/msg/reference_line.hpp>

namespace nuway_route {

struct ReferenceLineOptions {
  double spacing_m = 0.5;
  double blend_length_m = 30.0;
  double extension_m = 50.0;
  int curvature_window = 5;  // moving-average width, odd
};

// Builds the reference line message (header left empty) for an ordered lane
// list; nullopt when the list is empty or names unknown lanes.
std::optional<nuway_msgs::msg::ReferenceLine> BuildReferenceLine(
    const nuway_map::LaneGraph& graph,
    const std::vector<std::uint32_t>& lane_ids,
    const ReferenceLineOptions& options);

// Quintic ease 0 -> 1 with zero first and second derivatives at both ends.
double QuinticBlend(double t);

}  // namespace nuway_route

#endif  // NUWAY_ROUTE_REFERENCE_LINE_BUILDER_HPP_
