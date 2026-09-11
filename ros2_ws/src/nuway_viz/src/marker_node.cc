// marker_node: converts stack topics to the /nuway/viz/<layer> layers a
// Foxglove client draws (docs/02_interfaces.md §3.9, M1 §3.11). One node,
// many subscriptions; it is not part of the tick barrier and never blocks a
// producer. M0 gave it `lanes` and `reference_line`; M1 task 12 adds the
// agents (perception filled, GT hollow), the prediction samples faded by
// time, the lattice candidates colored by cost quantile, the selected and
// the safe trajectory, the MPC horizon, the reference line's drivable
// bounds and the occupancy raster.
//
// The lane graph and the reference line arrive latched. Viz QoS is
// best-effort/volatile, so a Foxglove client that connects later would never
// see them; both layers are therefore republished every
// `lanes_republish_ticks` messages of /nuway/loc/pose. That keys the repeat
// off sim ticks, not a wall-clock timer (docs/02_interfaces.md §2), and costs
// nothing in lockstep because this node is not part of the tick barrier.
//
// Perception agents arrive in base_link and are placed in the map with the
// EgoState of the same tick through AgentsToMap, the one helper every agent
// consumer uses (M1 §3.1), so the drawn box is where the planner saw it.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nuway_common/agents.h>
#include <nuway_common/frames.h>
#include <nuway_common/frenet.h>
#include <nuway_common/geometry.h>
#include <nuway_common/node_main.h>
#include <nuway_common/occupancy.h>
#include <nuway_common/params.h>
#include <nuway_common/qos.h>
#include <nuway_common/ros_conv.h>
#include <nuway_common/tick.h>
#include <nuway_msgs/msg/agent_array.hpp>
#include <nuway_msgs/msg/control_debug.hpp>
#include <nuway_msgs/msg/ego_state.hpp>
#include <nuway_msgs/msg/lane_graph.hpp>
#include <nuway_msgs/msg/occupancy_grid_mc.hpp>
#include <nuway_msgs/msg/prediction_samples.hpp>
#include <nuway_msgs/msg/reference_line.hpp>
#include <nuway_msgs/msg/trajectory.hpp>
#include <nuway_msgs/msg/trajectory_candidates.hpp>

#include "nuway_viz/names.h"

namespace nuway_viz {
namespace {

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

// Two trajectories are "the same plan" for the safe_trajectory layer when the
// source and candidate agree and the end points are within this distance:
// the safety layer re-times the plan by one tick on control-only ticks
// (metres at speed along the path, but the same end point), while a limits
// clip or a stop replaces the end point by metres.
constexpr double kSameTrajectoryEndM = 1.0;
// Ticks of EgoState kept for the agents' base_link -> map transform.
constexpr std::size_t kPoseHistory = 16;

std_msgs::msg::ColorRGBA Rgba(float r, float g, float b, float a) {
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

Marker BaseMarker(const std::string& ns, int id, std::int32_t type,
                  const builtin_interfaces::msg::Time& stamp) {
  Marker m;
  m.header.frame_id = nuway_common::kFrameMap;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.frame_locked = true;
  return m;
}

Marker LineStrip(const std::string& ns, int id, double width,
                 const std_msgs::msg::ColorRGBA& color,
                 const builtin_interfaces::msg::Time& stamp) {
  Marker m = BaseMarker(ns, id, Marker::LINE_STRIP, stamp);
  m.scale.x = width;
  m.color = color;
  return m;
}

Marker DeleteAll() {
  Marker clear;
  clear.action = Marker::DELETEALL;
  return clear;
}

geometry_msgs::msg::Point PointAt(double x, double y, double z) {
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// Box colour by agent class (the same hues as ml/nuway_ml/viz/style.py).
std_msgs::msg::ColorRGBA ClassColor(std::uint8_t class_id, float alpha) {
  switch (static_cast<nuway_common::AgentClass>(class_id)) {
    case nuway_common::AgentClass::kCar:
      return Rgba(0.25F, 0.55F, 1.0F, alpha);
    case nuway_common::AgentClass::kTruck:
      return Rgba(0.6F, 0.35F, 0.9F, alpha);
    case nuway_common::AgentClass::kBicycle:
    case nuway_common::AgentClass::kMotorcycle:
      return Rgba(0.1F, 0.85F, 0.85F, alpha);
    case nuway_common::AgentClass::kPedestrian:
      return Rgba(1.0F, 0.85F, 0.2F, alpha);
    case nuway_common::AgentClass::kStaticObstacle:
      return Rgba(0.6F, 0.6F, 0.6F, alpha);
    case nuway_common::AgentClass::kUnknown:
    default:
      return Rgba(0.95F, 0.95F, 0.95F, alpha);
  }
}

// Green -> yellow -> red over q in [0, 1]: the cost-quantile ramp of the
// candidates layer (the cheapest candidate green).
std_msgs::msg::ColorRGBA CostRamp(double q, float alpha) {
  const double t = std::clamp(q, 0.0, 1.0);
  const auto r = static_cast<float>(std::min(1.0, 2.0 * t));
  const auto g = static_cast<float>(std::min(1.0, 2.0 * (1.0 - t)));
  return Rgba(r, g, 0.1F, alpha);
}

// The four corners of an agent box (map frame, z at the box centre).
std::vector<geometry_msgs::msg::Point> BoxOutline(
    const nuway_msgs::msg::Agent& agent) {
  const nuway_common::SE2 pose = nuway_common::SE2FromMsg(agent.pose);
  const double hl = 0.5 * static_cast<double>(agent.length);
  const double hw = 0.5 * static_cast<double>(agent.width);
  const double z = agent.pose.position.z;
  std::vector<geometry_msgs::msg::Point> out;
  for (const auto& [dx, dy] :
       {std::pair{hl, hw}, std::pair{hl, -hw}, std::pair{-hl, -hw},
        std::pair{-hl, hw}, std::pair{hl, hw}}) {
    const Eigen::Vector2d p =
        nuway_common::Apply(pose, Eigen::Vector2d(dx, dy));
    out.push_back(PointAt(p.x(), p.y(), z));
  }
  // A nose tick from the centre to the front edge shows the heading.
  const Eigen::Vector2d c =
      nuway_common::Apply(pose, Eigen::Vector2d(0.0, 0.0));
  const Eigen::Vector2d n = nuway_common::Apply(pose, Eigen::Vector2d(hl, 0.0));
  out.push_back(PointAt(c.x(), c.y(), z));
  out.push_back(PointAt(n.x(), n.y(), z));
  return out;
}

// Builds the whole `lanes` layer: lane centerlines colored by type, stop
// lines of traffic lights and stop signs, crosswalk footprints.
MarkerArray LanesLayer(const nuway_msgs::msg::LaneGraph& graph,
                       const builtin_interfaces::msg::Time& stamp) {
  MarkerArray out;
  out.markers.push_back(DeleteAll());
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
    Marker m =
        LineStrip("lanes", id++, 0.15, drivable ? driving : other, stamp);
    m.points = lane.centerline;
    out.markers.push_back(std::move(m));
  }
  const auto stop_line = [&](const geometry_msgs::msg::Point& center,
                             float heading,
                             const std_msgs::msg::ColorRGBA& color) {
    // A 3.5 m bar across the driving direction (heading faces the traffic).
    Marker m = LineStrip("stop_lines", id++, 0.3, color, stamp);
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
    Marker m = LineStrip("stop_signs", id++, 0.2, stop, stamp);
    for (const geometry_msgs::msg::Point32& p : sign.trigger_volume.points) {
      m.points.push_back(PointAt(static_cast<double>(p.x),
                                 static_cast<double>(p.y), sign.stop_line.z));
    }
    if (!m.points.empty()) {
      m.points.push_back(m.points.front());
    }
    out.markers.push_back(std::move(m));
  }
  for (const nuway_msgs::msg::Crosswalk& cw : graph.crosswalks) {
    Marker m = LineStrip("crosswalks", id++, 0.2, crosswalk, stamp);
    for (const geometry_msgs::msg::Point32& p : cw.footprint.points) {
      m.points.push_back(
          PointAt(static_cast<double>(p.x), static_cast<double>(p.y), 0.0));
    }
    if (!m.points.empty()) {
      m.points.push_back(m.points.front());
    }
    out.markers.push_back(std::move(m));
  }
  return out;
}

// The static part of the `reference_line` layer: the line and its drivable
// bounds (the left/right edge distances of each sample, along the normal).
MarkerArray ReferenceLineLayer(const nuway_msgs::msg::ReferenceLine& line,
                               const builtin_interfaces::msg::Time& stamp) {
  MarkerArray out;
  Marker m =
      LineStrip("reference_line", 0, 0.4, Rgba(1.0F, 0.5F, 0.0F, 0.9F), stamp);
  m.points = line.points;
  for (geometry_msgs::msg::Point& p : m.points) {
    p.z += 0.1;
  }
  out.markers.push_back(std::move(m));
  Marker left =
      LineStrip("bounds", 2, 0.12, Rgba(1.0F, 0.75F, 0.3F, 0.6F), stamp);
  Marker right =
      LineStrip("bounds", 3, 0.12, Rgba(1.0F, 0.75F, 0.3F, 0.6F), stamp);
  const std::size_t n =
      std::min({line.points.size(), line.heading.size(), line.left_bound.size(),
                line.right_bound.size()});
  for (std::size_t i = 0; i < n; ++i) {
    const double nx = -std::sin(static_cast<double>(line.heading[i]));
    const double ny = std::cos(static_cast<double>(line.heading[i]));
    const geometry_msgs::msg::Point& p = line.points[i];
    const auto l = static_cast<double>(line.left_bound[i]);
    const auto r = static_cast<double>(line.right_bound[i]);
    left.points.push_back(PointAt(p.x + (l * nx), p.y + (l * ny), p.z + 0.1));
    right.points.push_back(PointAt(p.x - (r * nx), p.y - (r * ny), p.z + 0.1));
  }
  if (n >= 2) {
    out.markers.push_back(std::move(left));
    out.markers.push_back(std::move(right));
  }
  return out;
}

// The per-tick part: the pure pursuit lookahead point (M0; the MPC reports
// a zero lookahead and draws its horizon instead).
Marker LookaheadMarker(const nuway_common::CartesianPoint& point,
                       const builtin_interfaces::msg::Time& stamp) {
  Marker m = BaseMarker("lookahead", 1, Marker::SPHERE, stamp);
  m.pose.position.x = point.x;
  m.pose.position.y = point.y;
  m.pose.position.z = 0.5;
  m.scale.x = 0.8;
  m.scale.y = 0.8;
  m.scale.z = 0.8;
  m.color = Rgba(0.0F, 0.9F, 1.0F, 1.0F);
  return m;
}

// Agent boxes in the map frame: filled cubes (perception) or hollow
// outlines (GT, drawn so the two can be told apart when both are on), each
// with an "id v" label above it.
MarkerArray AgentsLayer(const nuway_msgs::msg::AgentArray& agents,
                        const builtin_interfaces::msg::Time& stamp,
                        bool hollow) {
  MarkerArray out;
  out.markers.push_back(DeleteAll());
  int id = 0;
  for (const nuway_msgs::msg::Agent& agent : agents.agents) {
    const float alpha = agent.visible ? 0.75F : 0.35F;
    if (hollow) {
      Marker m = LineStrip("boxes", id++, 0.12,
                           ClassColor(agent.class_id, alpha), stamp);
      m.points = BoxOutline(agent);
      out.markers.push_back(std::move(m));
    } else {
      Marker m = BaseMarker("boxes", id++, Marker::CUBE, stamp);
      m.pose = agent.pose;
      m.scale.x = std::max(0.3, static_cast<double>(agent.length));
      m.scale.y = std::max(0.3, static_cast<double>(agent.width));
      m.scale.z = std::max(0.3, static_cast<double>(agent.height));
      m.color = ClassColor(agent.class_id, alpha);
      const double roof = 0.5 * m.scale.z;
      out.markers.push_back(std::move(m));
      Marker nose =
          LineStrip("heading", id++, 0.1, Rgba(1.0F, 1.0F, 1.0F, 0.9F), stamp);
      const std::vector<geometry_msgs::msg::Point> outline = BoxOutline(agent);
      nose.points = {outline[outline.size() - 2], outline.back()};
      nose.points[0].z += roof;
      nose.points[1].z += roof;
      out.markers.push_back(std::move(nose));
    }
    Marker label = BaseMarker("labels", id++, Marker::TEXT_VIEW_FACING, stamp);
    label.pose.position = agent.pose.position;
    label.pose.position.z += static_cast<double>(agent.height) + 0.8;
    label.scale.z = 0.7;
    label.color = Rgba(1.0F, 1.0F, 1.0F, 0.9F);
    const double speed = std::hypot(static_cast<double>(agent.vx),
                                    static_cast<double>(agent.vy));
    char text[48];
    std::snprintf(text, sizeof(text), "%u %.1f", agent.id, speed);
    label.text = text;
    out.markers.push_back(std::move(label));
  }
  return out;
}

// Prediction samples: one polyline per (sample, agent) whose alpha fades
// with the step, so the near future is bold and the 8 s tail faint; the
// sample weight scales the width.
MarkerArray PredictionsLayer(const nuway_msgs::msg::PredictionSamples& samples,
                             const builtin_interfaces::msg::Time& stamp) {
  MarkerArray out;
  out.markers.push_back(DeleteAll());
  const std::size_t s_n = samples.num_samples;
  const std::size_t a_n = samples.agent_ids.size();
  const std::size_t t_n = samples.num_timesteps;
  if (samples.xy.size() < s_n * a_n * t_n * 2) {
    return out;
  }
  int id = 0;
  for (std::size_t s = 0; s < s_n; ++s) {
    const double weight = s < samples.sample_weight.size()
                              ? static_cast<double>(samples.sample_weight[s])
                              : 1.0;
    for (std::size_t a = 0; a < a_n; ++a) {
      Marker m = LineStrip("samples", id++, 0.06 + (0.2 * weight),
                           Rgba(0.9F, 0.3F, 0.9F, 0.8F), stamp);
      for (std::size_t t = 0; t < t_n; ++t) {
        const std::size_t base = ((((s * a_n) + a) * t_n) + t) * 2;
        m.points.push_back(PointAt(static_cast<double>(samples.xy[base]),
                                   static_cast<double>(samples.xy[base + 1]),
                                   0.15));
        const float fade =
            0.9F - (0.8F * static_cast<float>(t) /
                    static_cast<float>(std::max<std::size_t>(1, t_n - 1)));
        m.colors.push_back(Rgba(0.9F, 0.3F, 0.9F, fade));
      }
      out.markers.push_back(std::move(m));
    }
  }
  return out;
}

Marker TrajectoryStrip(const nuway_msgs::msg::Trajectory& traj,
                       const std::string& ns, int id, double width,
                       const std_msgs::msg::ColorRGBA& color, double z,
                       const builtin_interfaces::msg::Time& stamp) {
  Marker m = LineStrip(ns, id, width, color, stamp);
  for (const nuway_msgs::msg::TrajectoryPoint& p : traj.points) {
    m.points.push_back(
        PointAt(static_cast<double>(p.x), static_cast<double>(p.y), z));
  }
  return m;
}

// Lattice candidates, thin, colored by cost quantile (rank over the set).
MarkerArray CandidatesLayer(const nuway_msgs::msg::TrajectoryCandidates& cands,
                            const builtin_interfaces::msg::Time& stamp) {
  MarkerArray out;
  out.markers.push_back(DeleteAll());
  const std::size_t n = std::min(cands.candidates.size(), cands.cost.size());
  std::vector<std::size_t> order(n);
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    return cands.cost[a] < cands.cost[b];
  });
  std::vector<double> quantile(n, 0.0);
  for (std::size_t rank = 0; rank < n; ++rank) {
    quantile[order[rank]] =
        n > 1 ? static_cast<double>(rank) / static_cast<double>(n - 1) : 0.0;
  }
  for (std::size_t i = 0; i < n; ++i) {
    out.markers.push_back(
        TrajectoryStrip(cands.candidates[i], "candidates", static_cast<int>(i),
                        0.06, CostRamp(quantile[i], 0.55F), 0.12, stamp));
  }
  return out;
}

// The selected / safe trajectory: a thick strip plus a sphere every second.
MarkerArray TrajectoryLayer(const nuway_msgs::msg::Trajectory& traj,
                            const std_msgs::msg::ColorRGBA& color, double z,
                            const builtin_interfaces::msg::Time& stamp) {
  MarkerArray out;
  out.markers.push_back(DeleteAll());
  out.markers.push_back(TrajectoryStrip(traj, "path", 0, 0.3, color, z, stamp));
  Marker dots = BaseMarker("seconds", 1, Marker::SPHERE_LIST, stamp);
  dots.scale.x = 0.5;
  dots.scale.y = 0.5;
  dots.scale.z = 0.5;
  dots.color = color;
  for (std::size_t i = 0; i < traj.points.size(); i += 10) {
    dots.points.push_back(PointAt(static_cast<double>(traj.points[i].x),
                                  static_cast<double>(traj.points[i].y), z));
  }
  out.markers.push_back(std::move(dots));
  return out;
}

bool SamePlan(const nuway_msgs::msg::Trajectory& a,
              const nuway_msgs::msg::Trajectory& b) {
  if (a.source != b.source || a.candidate_id != b.candidate_id ||
      a.points.empty() || b.points.empty()) {
    return false;
  }
  const double dx = static_cast<double>(a.points.back().x) -
                    static_cast<double>(b.points.back().x);
  const double dy = static_cast<double>(a.points.back().y) -
                    static_cast<double>(b.points.back().y);
  return std::hypot(dx, dy) < kSameTrajectoryEndM;
}

// The occupancy raster (docs/02 §3.9): `occupied` red, `free` white,
// `unknown` grey, `drivable` tint, `dynamic` blue, in that priority. The
// grid is base_link with rows along x (forward) and columns along y (left);
// the image puts forward at the top and left on the left, as a BEV is read.
sensor_msgs::msg::Image OccupancyImage(
    const nuway_msgs::msg::OccupancyGridMC& grid) {
  sensor_msgs::msg::Image img;
  img.header = grid.header;
  img.height = grid.height;
  img.width = grid.width;
  img.encoding = "rgb8";
  img.is_bigendian = 0;
  img.step = static_cast<std::uint32_t>(grid.width) * 3U;
  const std::size_t cells = static_cast<std::size_t>(grid.height) * grid.width;
  img.data.assign(cells * 3, 0);
  if (grid.data.size() < cells * grid.num_channels || grid.num_channels < 6) {
    return img;
  }
  const auto channel = [&](std::size_t c, std::size_t cell) {
    return grid.data[(c * cells) + cell];
  };
  for (std::size_t r = 0; r < grid.height; ++r) {
    for (std::size_t col = 0; col < grid.width; ++col) {
      const std::size_t cell = (r * grid.width) + col;
      std::uint8_t red = 255;
      std::uint8_t green = 255;
      std::uint8_t blue = 255;
      if (channel(3, cell) > 0.5F) {  // drivable tint
        red = 225;
        green = 240;
        blue = 225;
      }
      if (channel(2, cell) > 0.5F) {  // unknown
        red = 150;
        green = 150;
        blue = 150;
      }
      if (channel(5, cell) > 0.5F) {  // dynamic
        red = 60;
        green = 120;
        blue = 255;
      }
      if (channel(0, cell) > 0.5F) {  // occupied
        red = 220;
        green = 40;
        blue = 40;
      }
      const std::size_t out_row = static_cast<std::size_t>(grid.height) - 1 - r;
      const std::size_t out_col =
          static_cast<std::size_t>(grid.width) - 1 - col;
      const std::size_t o = ((out_row * grid.width) + out_col) * 3;
      img.data[o] = red;
      img.data[o + 1] = green;
      img.data[o + 2] = blue;
    }
  }
  return img;
}

class MarkerNode final : public rclcpp::Node {
 public:
  MarkerNode() : rclcpp::Node(kNodeName) {
    lanes_republish_ticks_ = nuway_common::DeclareParam<int>(
        this, "lanes_republish_ticks", 200,
        "republish the static layers every N /nuway/loc/pose messages");
    const auto layer = [this](const char* name) {
      return create_publisher<MarkerArray>(std::string(kTopicVizPrefix) + name,
                                           nuway_common::qos::Viz());
    };
    pub_lanes_ = layer(kLayerLanes);
    pub_reference_line_ = layer(kLayerReferenceLine);
    pub_agents_ = layer(kLayerAgents);
    pub_gt_agents_ = layer(kLayerGtAgents);
    pub_predictions_ = layer(kLayerPredictions);
    pub_candidates_ = layer(kLayerCandidates);
    pub_trajectory_ = layer(kLayerTrajectory);
    pub_safe_trajectory_ = layer(kLayerSafeTrajectory);
    pub_mpc_horizon_ = layer(kLayerMpcHorizon);
    pub_occupancy_ = create_publisher<sensor_msgs::msg::Image>(
        std::string(kTopicVizPrefix) + kLayerOccupancy,
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
    sub_agents_ = create_subscription<nuway_msgs::msg::AgentArray>(
        nuway_common::kTopicPerceptionAgents, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::AgentArray::ConstSharedPtr msg) {
          pub_agents_->publish(
              AgentsLayer(ToMap(*msg), msg->header.stamp, false));
        });
    sub_gt_agents_ = create_subscription<nuway_msgs::msg::AgentArray>(
        nuway_common::kTopicGtAgents, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::AgentArray::ConstSharedPtr msg) {
          pub_gt_agents_->publish(
              AgentsLayer(ToMap(*msg), msg->header.stamp, true));
        });
    sub_predictions_ = create_subscription<nuway_msgs::msg::PredictionSamples>(
        nuway_common::kTopicPredictionSamples, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::PredictionSamples::ConstSharedPtr msg) {
          pub_predictions_->publish(PredictionsLayer(*msg, msg->header.stamp));
        });
    sub_candidates_ =
        create_subscription<nuway_msgs::msg::TrajectoryCandidates>(
            nuway_common::kTopicPlanningCandidates, nuway_common::qos::Stream(),
            [this](nuway_msgs::msg::TrajectoryCandidates::ConstSharedPtr msg) {
              pub_candidates_->publish(
                  CandidatesLayer(*msg, msg->header.stamp));
            });
    sub_trajectory_ = create_subscription<nuway_msgs::msg::Trajectory>(
        nuway_common::kTopicPlanningTrajectory, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::Trajectory::ConstSharedPtr msg) {
          OnTrajectory(std::move(msg));
        });
    sub_safe_trajectory_ = create_subscription<nuway_msgs::msg::Trajectory>(
        nuway_common::kTopicPlanningSafeTrajectory, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::Trajectory::ConstSharedPtr msg) {
          OnSafeTrajectory(*msg);
        });
    sub_horizon_ = create_subscription<nuway_msgs::msg::Trajectory>(
        nuway_common::kTopicControlHorizon, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::Trajectory::ConstSharedPtr msg) {
          pub_mpc_horizon_->publish(TrajectoryLayer(
              *msg, Rgba(0.2F, 1.0F, 0.4F, 0.9F), 0.3, msg->header.stamp));
        });
    sub_occupancy_ = create_subscription<nuway_msgs::msg::OccupancyGridMC>(
        nuway_common::kTopicPerceptionOccupancy, nuway_common::qos::Stream(),
        [this](nuway_msgs::msg::OccupancyGridMC::ConstSharedPtr msg) {
          pub_occupancy_->publish(OccupancyImage(*msg));
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
    poses_[nuway_common::TickIndex(pose.header.stamp)] = pose;
    while (poses_.size() > kPoseHistory) {
      poses_.erase(poses_.begin());
    }
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

  // Agents into the map frame with the EgoState of their own tick (the
  // latest pose when that tick's is not held, e.g. a bag played without
  // /nuway/loc/pose); a map-frame array passes through.
  nuway_msgs::msg::AgentArray ToMap(const nuway_msgs::msg::AgentArray& agents) {
    if (agents.header.frame_id == nuway_common::kFrameMap) {
      return agents;
    }
    const auto it = poses_.find(nuway_common::TickIndex(agents.header.stamp));
    const nuway_msgs::msg::EgoState& ego =
        it != poses_.end() ? it->second : last_pose_;
    if (!ego.valid) {
      return nuway_msgs::msg::AgentArray();
    }
    return nuway_common::AgentsToMap(agents, ego);
  }

  // Draws the lookahead point at s_ego + lookahead on the line, for the
  // tick the debug message answers (pure pursuit only: the MPC reports 0).
  void OnControlDebug(const nuway_msgs::msg::ControlDebug& debug) {
    if (reference_line_ == nullptr || !last_pose_.valid || !debug.solver_ok ||
        debug.lookahead <= 0.0F) {
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

  void OnTrajectory(nuway_msgs::msg::Trajectory::ConstSharedPtr msg) {
    trajectory_ = std::move(msg);
    pub_trajectory_->publish(TrajectoryLayer(*trajectory_,
                                             Rgba(0.1F, 0.6F, 1.0F, 0.95F), 0.2,
                                             trajectory_->header.stamp));
  }

  // The safe trajectory is drawn only where it differs from the plan (a
  // safety-layer intervention or the no-input stop), red and above it;
  // otherwise the layer is cleared so a stale intervention does not linger.
  void OnSafeTrajectory(const nuway_msgs::msg::Trajectory& safe) {
    if (trajectory_ != nullptr && SamePlan(*trajectory_, safe)) {
      MarkerArray clear;
      clear.markers.push_back(DeleteAll());
      pub_safe_trajectory_->publish(clear);
      return;
    }
    pub_safe_trajectory_->publish(TrajectoryLayer(
        safe, Rgba(1.0F, 0.15F, 0.15F, 0.95F), 0.25, safe.header.stamp));
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
  nuway_msgs::msg::Trajectory::ConstSharedPtr trajectory_;
  nuway_common::ReferenceLine frenet_;
  nuway_msgs::msg::EgoState last_pose_;
  std::map<std::int64_t, nuway_msgs::msg::EgoState> poses_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_lanes_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_reference_line_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_agents_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_gt_agents_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_predictions_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_candidates_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_safe_trajectory_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_mpc_horizon_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_occupancy_;
  rclcpp::Subscription<nuway_msgs::msg::LaneGraph>::SharedPtr sub_lane_graph_;
  rclcpp::Subscription<nuway_msgs::msg::ReferenceLine>::SharedPtr
      sub_reference_line_;
  rclcpp::Subscription<nuway_msgs::msg::EgoState>::SharedPtr sub_pose_;
  rclcpp::Subscription<nuway_msgs::msg::ControlDebug>::SharedPtr
      sub_control_debug_;
  rclcpp::Subscription<nuway_msgs::msg::AgentArray>::SharedPtr sub_agents_;
  rclcpp::Subscription<nuway_msgs::msg::AgentArray>::SharedPtr sub_gt_agents_;
  rclcpp::Subscription<nuway_msgs::msg::PredictionSamples>::SharedPtr
      sub_predictions_;
  rclcpp::Subscription<nuway_msgs::msg::TrajectoryCandidates>::SharedPtr
      sub_candidates_;
  rclcpp::Subscription<nuway_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<nuway_msgs::msg::Trajectory>::SharedPtr
      sub_safe_trajectory_;
  rclcpp::Subscription<nuway_msgs::msg::Trajectory>::SharedPtr sub_horizon_;
  rclcpp::Subscription<nuway_msgs::msg::OccupancyGridMC>::SharedPtr
      sub_occupancy_;
};

}  // namespace
}  // namespace nuway_viz

int main(int argc, char** argv) {
  return nuway_common::RunNode<nuway_viz::MarkerNode>(argc, argv);
}
