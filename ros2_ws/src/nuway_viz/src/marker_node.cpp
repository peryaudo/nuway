// marker_node (M0 tasks 7 and 12, minimal): converts stack topics to
// /nuway/viz/<layer> MarkerArray layers for Foxglove (docs/02_interfaces.md
// §3.9). M0 provides `lanes` and `reference_line` (the line plus the pure
// pursuit lookahead point from /nuway/control/debug); M1 task 12 adds the
// rest.
//
// The lane graph and the reference line arrive latched. Viz QoS is
// best-effort/volatile, so a Foxglove client that connects later would never
// see them; both layers are therefore republished every
// `lanes_republish_ticks` messages of /nuway/loc/pose. That keys the repeat
// off sim ticks, not a wall-clock timer (docs/02_interfaces.md §2), and costs
// nothing in lockstep because this node is not part of the tick barrier.
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nuway_common/frames.hpp>
#include <nuway_common/frenet.hpp>
#include <nuway_common/geometry.hpp>
#include <nuway_common/node_main.hpp>
#include <nuway_common/params.hpp>
#include <nuway_common/qos.hpp>
#include <nuway_common/ros_conv.hpp>
#include <nuway_msgs/msg/control_debug.hpp>
#include <nuway_msgs/msg/ego_state.hpp>
#include <nuway_msgs/msg/lane_graph.hpp>
#include <nuway_msgs/msg/reference_line.hpp>

#include "nuway_viz/names.hpp"

namespace nuway_viz {
namespace {

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

std_msgs::msg::ColorRGBA Rgba(float r, float g, float b, float a) {
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

Marker LineStrip(const std::string& ns, int id, double width,
                 const std_msgs::msg::ColorRGBA& color) {
  Marker m;
  m.header.frame_id = nuway_common::kFrameMap;
  m.ns = ns;
  m.id = id;
  m.type = Marker::LINE_STRIP;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = width;
  m.color = color;
  m.frame_locked = true;
  return m;
}

// Builds the whole `lanes` layer: lane centerlines colored by type, stop
// lines of traffic lights and stop signs, crosswalk footprints.
MarkerArray LanesLayer(const nuway_msgs::msg::LaneGraph& graph,
                       const builtin_interfaces::msg::Time& stamp) {
  MarkerArray out;
  Marker clear;
  clear.action = Marker::DELETEALL;
  out.markers.push_back(clear);
  const std_msgs::msg::ColorRGBA driving = Rgba(0.55F, 0.55F, 0.6F, 0.8F);
  const std_msgs::msg::ColorRGBA other = Rgba(0.4F, 0.45F, 0.35F, 0.5F);
  const std_msgs::msg::ColorRGBA light = Rgba(1.0F, 0.2F, 0.2F, 0.9F);
  const std_msgs::msg::ColorRGBA stop = Rgba(1.0F, 0.6F, 0.1F, 0.9F);
  const std_msgs::msg::ColorRGBA crosswalk = Rgba(0.9F, 0.9F, 0.9F, 0.7F);
  int id = 0;
  for (const nuway_msgs::msg::Lane& lane : graph.lanes) {
    const bool drivable =
        lane.type == nuway_msgs::msg::Lane::TYPE_DRIVING ||
        lane.type == nuway_msgs::msg::Lane::TYPE_BIDIRECTIONAL;
    Marker m = LineStrip("lanes", id++, 0.15, drivable ? driving : other);
    m.header.stamp = stamp;
    m.points = lane.centerline;
    out.markers.push_back(std::move(m));
  }
  const auto stop_line = [&](const geometry_msgs::msg::Point& center,
                             float heading,
                             const std_msgs::msg::ColorRGBA& color) {
    // A 3.5 m bar across the driving direction (heading faces the traffic).
    Marker m = LineStrip("stop_lines", id++, 0.3, color);
    m.header.stamp = stamp;
    const double nx = -std::sin(static_cast<double>(heading));
    const double ny = std::cos(static_cast<double>(heading));
    geometry_msgs::msg::Point a = center;
    geometry_msgs::msg::Point b = center;
    a.x += 1.75 * nx;
    a.y += 1.75 * ny;
    b.x -= 1.75 * nx;
    b.y -= 1.75 * ny;
    m.points = {a, b};
    out.markers.push_back(std::move(m));
  };
  for (const nuway_msgs::msg::TrafficLightMapping& tl : graph.traffic_lights) {
    stop_line(tl.stop_line, tl.heading, light);
  }
  for (const nuway_msgs::msg::StopSign& sign : graph.stop_signs) {
    Marker m = LineStrip("stop_signs", id++, 0.2, stop);
    m.header.stamp = stamp;
    for (const geometry_msgs::msg::Point32& p : sign.trigger_volume.points) {
      geometry_msgs::msg::Point q;
      q.x = static_cast<double>(p.x);
      q.y = static_cast<double>(p.y);
      q.z = sign.stop_line.z;
      m.points.push_back(q);
    }
    if (!m.points.empty()) {
      m.points.push_back(m.points.front());
    }
    out.markers.push_back(std::move(m));
  }
  for (const nuway_msgs::msg::Crosswalk& cw : graph.crosswalks) {
    Marker m = LineStrip("crosswalks", id++, 0.2, crosswalk);
    m.header.stamp = stamp;
    for (const geometry_msgs::msg::Point32& p : cw.footprint.points) {
      geometry_msgs::msg::Point q;
      q.x = static_cast<double>(p.x);
      q.y = static_cast<double>(p.y);
      q.z = 0.0;
      m.points.push_back(q);
    }
    if (!m.points.empty()) {
      m.points.push_back(m.points.front());
    }
    out.markers.push_back(std::move(m));
  }
  return out;
}

// The static part of the `reference_line` layer: the line itself.
MarkerArray ReferenceLineLayer(const nuway_msgs::msg::ReferenceLine& line,
                               const builtin_interfaces::msg::Time& stamp) {
  MarkerArray out;
  Marker m = LineStrip("reference_line", 0, 0.4, Rgba(1.0F, 0.5F, 0.0F, 0.9F));
  m.header.stamp = stamp;
  m.points = line.points;
  for (geometry_msgs::msg::Point& p : m.points) {
    p.z += 0.1;
  }
  out.markers.push_back(std::move(m));
  return out;
}

// The per-tick part: the pure pursuit lookahead point.
Marker LookaheadMarker(const nuway_common::CartesianPoint& point,
                       const builtin_interfaces::msg::Time& stamp) {
  Marker m;
  m.header.frame_id = nuway_common::kFrameMap;
  m.header.stamp = stamp;
  m.ns = "lookahead";
  m.id = 1;
  m.type = Marker::SPHERE;
  m.action = Marker::ADD;
  m.pose.position.x = point.x;
  m.pose.position.y = point.y;
  m.pose.position.z = 0.5;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.8;
  m.scale.y = 0.8;
  m.scale.z = 0.8;
  m.color = Rgba(0.0F, 0.9F, 1.0F, 1.0F);
  m.frame_locked = true;
  return m;
}

class MarkerNode final : public rclcpp::Node {
 public:
  MarkerNode() : rclcpp::Node(kNodeName) {
    lanes_republish_ticks_ = nuway_common::DeclareParam<int>(
        this, "lanes_republish_ticks", 200,
        "republish the static layers every N /nuway/loc/pose messages");
    pub_lanes_ = create_publisher<MarkerArray>(
        std::string(kTopicVizPrefix) + kLayerLanes, nuway_common::qos::Viz());
    pub_reference_line_ = create_publisher<MarkerArray>(
        std::string(kTopicVizPrefix) + kLayerReferenceLine,
        nuway_common::qos::Viz());
    sub_lane_graph_ = create_subscription<nuway_msgs::msg::LaneGraph>(
        nuway_common::kTopicLaneGraph, nuway_common::qos::Latched(),
        [this](nuway_msgs::msg::LaneGraph::ConstSharedPtr msg) {
          OnLaneGraph(std::move(msg));
        });
    sub_reference_line_ = create_subscription<nuway_msgs::msg::ReferenceLine>(
        nuway_common::kTopicReferenceLine, nuway_common::qos::Latched(),
        [this](nuway_msgs::msg::ReferenceLine::ConstSharedPtr msg) {
          OnReferenceLine(std::move(msg));
        });
    sub_pose_ = create_subscription<nuway_msgs::msg::EgoState>(
        nuway_common::kTopicPose, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::EgoState::ConstSharedPtr msg) {
          OnPose(*msg);
        });
    sub_control_debug_ = create_subscription<nuway_msgs::msg::ControlDebug>(
        nuway_common::kTopicControlDebug, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::ControlDebug::ConstSharedPtr msg) {
          OnControlDebug(*msg);
        });
  }

 private:
  void OnLaneGraph(nuway_msgs::msg::LaneGraph::ConstSharedPtr msg) {
    lane_graph_ = std::move(msg);
    RCLCPP_INFO(get_logger(), "lane graph: %zu lanes",
                lane_graph_->lanes.size());
    PublishLanes(lane_graph_->header.stamp);
  }

  void OnReferenceLine(nuway_msgs::msg::ReferenceLine::ConstSharedPtr msg) {
    reference_line_ = std::move(msg);
    nuway_common::Vector2dList points;
    points.reserve(reference_line_->points.size());
    for (const geometry_msgs::msg::Point& p : reference_line_->points) {
      points.emplace_back(p.x, p.y);
    }
    frenet_ = nuway_common::ReferenceLine::FromPoints(points);
    PublishReferenceLine(reference_line_->header.stamp);
  }

  void OnPose(const nuway_msgs::msg::EgoState& pose) {
    ++pose_count_;
    last_pose_ = pose;
    if (lanes_republish_ticks_ > 0 &&
        pose_count_ % static_cast<std::uint64_t>(lanes_republish_ticks_) == 0) {
      if (lane_graph_ != nullptr) {
        PublishLanes(pose.header.stamp);
      }
      if (reference_line_ != nullptr) {
        PublishReferenceLine(pose.header.stamp);
      }
    }
  }

  // Draws the lookahead point at s_ego + lookahead on the line, for the
  // tick the debug message answers.
  void OnControlDebug(const nuway_msgs::msg::ControlDebug& debug) {
    if (reference_line_ == nullptr || !last_pose_.valid || !debug.solver_ok) {
      return;
    }
    const nuway_common::SE2 ego = nuway_common::SE2FromMsg(last_pose_.pose);
    const std::optional<nuway_common::FrenetPoint> f =
        frenet_.ToFrenet(ego.x, ego.y, 10.0);
    if (!f.has_value()) {
      return;
    }
    MarkerArray out;
    out.markers.push_back(LookaheadMarker(
        frenet_.PointAt(f->s + static_cast<double>(debug.lookahead)),
        debug.header.stamp));
    pub_reference_line_->publish(out);
  }

  void PublishLanes(const builtin_interfaces::msg::Time& stamp) {
    pub_lanes_->publish(LanesLayer(*lane_graph_, stamp));
  }

  void PublishReferenceLine(const builtin_interfaces::msg::Time& stamp) {
    pub_reference_line_->publish(ReferenceLineLayer(*reference_line_, stamp));
  }

  int lanes_republish_ticks_ = 200;
  std::uint64_t pose_count_ = 0;
  nuway_msgs::msg::LaneGraph::ConstSharedPtr lane_graph_;
  nuway_msgs::msg::ReferenceLine::ConstSharedPtr reference_line_;
  nuway_common::ReferenceLine frenet_;
  nuway_msgs::msg::EgoState last_pose_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_lanes_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_reference_line_;
  rclcpp::Subscription<nuway_msgs::msg::LaneGraph>::SharedPtr sub_lane_graph_;
  rclcpp::Subscription<nuway_msgs::msg::ReferenceLine>::SharedPtr
      sub_reference_line_;
  rclcpp::Subscription<nuway_msgs::msg::EgoState>::SharedPtr sub_pose_;
  rclcpp::Subscription<nuway_msgs::msg::ControlDebug>::SharedPtr
      sub_control_debug_;
};

}  // namespace
}  // namespace nuway_viz

int main(int argc, char** argv) {
  return nuway_common::RunNode<nuway_viz::MarkerNode>(argc, argv);
}
