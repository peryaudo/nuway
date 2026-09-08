// route_planner_node (M0 §2.5): subscribes the latched lane graph,
// /nuway/loc/pose and /nuway/route/waypoints (the whole route, once per
// episode), plans A* through every waypoint once and publishes
// /nuway/route/plan and /nuway/route/reference_line latched. Republishes only
// on reroute: ego more than reroute_lateral_m off the line or
// reroute_heading_rad heading-off for reroute_ticks consecutive poses,
// replanned from the current lane through the remaining waypoints.
//
// Lockstep contract (docs/02_interfaces.md §2). The stack advances one tick
// at a time and every node keys off message stamps, never wall-clock timers.
// This node is *not* part of the per-tick barrier: the route and reference
// line are latest-value inputs of the consumers (latched QoS, so a late
// subscriber still gets the current one), and it publishes them only when
// they change. It still acts on every pose: each /nuway/loc/pose triggers
// one planning attempt or one reroute check and one diag message stamped
// with that pose, so the tick spacing is what paces it. The no-input
// convention is what keeps the gate alive while this node has nothing yet:
// the controller answers a tick without a reference line with an
// emergency_stop stamped for that tick, so the first ticks of an episode
// (before the waypoints arrive and the plan is made) are not timeouts.
//
// Episodes and reset. Cross-cycle state (route, reference line, off-line
// tick counter) is dropped on /nuway/sim/reset_event; waypoints and poses
// stamped before the reset are ignored (docs/02_interfaces.md §7). "Before"
// is decided on tick indices (TickIndex(stamp), round(stamp / 0.05 s)), not
// raw stamps: the reset event and the first pose of an episode are stamped
// by different publishers whose floating-point stamps of one tick differ by
// nanoseconds. The reset topic is transient_local, so an old event can be
// replayed to a late subscriber; only an event with a newer episode id
// clears anything (Decisions log, task 8 review fixes).
//
// Reroute detection. Every pose is projected onto the current line
// (Frenet s, d); it is "off" when the projection fails, |d| exceeds
// reroute_lateral_m or the heading error exceeds reroute_heading_rad. A
// counter of consecutive off poses must reach reroute_ticks (2 s at 20 Hz)
// before a replan, so a single bad tick does not swap the line under the
// controller. The replan starts from the first waypoint not yet passed,
// judged by the s each waypoint was projected to when the line was built.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nuway_common/diag.h>
#include <nuway_common/frames.h>
#include <nuway_common/frenet.h>
#include <nuway_common/geometry.h>
#include <nuway_common/node_main.h>
#include <nuway_common/params.h>
#include <nuway_common/qos.h>
#include <nuway_common/ros_conv.h>
#include <nuway_common/tick.h>
#include <nuway_map/lane_graph.h>
#include <nuway_msgs/msg/ego_state.hpp>
#include <nuway_msgs/msg/lane_graph.hpp>
#include <nuway_msgs/msg/reference_line.hpp>
#include <nuway_msgs/msg/reset_event.hpp>
#include <nuway_msgs/msg/route.hpp>

#include "nuway_route/names.h"
#include "nuway_route/reference_line_builder.h"
#include "nuway_route/route_planner.h"

namespace nuway_route {
namespace {

using nuway_common::DiagStatus;

class RoutePlannerNode final : public rclcpp::Node {
 public:
  // Declares the parameters and wires the topics. Subscriptions are created
  // in the order the executor serves them within one wake (see
  // OnResetEvent for why waypoints come before the reset event).
  RoutePlannerNode() : rclcpp::Node(kNodeName), diag_(this) {
    planner_options_.lane_change_penalty_m = nuway_common::DeclareParam<double>(
        this, "lane_change_penalty_m", 20.0, "A* cost of a lateral edge");
    planner_options_.waypoint_max_dist_m = nuway_common::DeclareParam<double>(
        this, "waypoint_max_dist_m", 10.0, "max waypoint-to-lane distance");
    planner_options_.ego_max_dist_m = nuway_common::DeclareParam<double>(
        this, "ego_max_dist_m", 10.0, "max ego-to-lane distance");
    line_options_.spacing_m = nuway_common::DeclareParam<double>(
        this, "spacing_m", 0.5, "reference line spacing");
    line_options_.blend_length_m = nuway_common::DeclareParam<double>(
        this, "blend_length_m", 30.0, "lane-change blend length");
    line_options_.extension_m = nuway_common::DeclareParam<double>(
        this, "extension_m", 50.0, "extension past the goal");
    reroute_lateral_m_ = nuway_common::DeclareParam<double>(
        this, "reroute_lateral_m", 5.0,
        "lateral offset that triggers a reroute");
    reroute_heading_rad_ = nuway_common::DeclareParam<double>(
        this, "reroute_heading_rad", 0.5236,
        "heading error that triggers a reroute");
    reroute_ticks_ = nuway_common::DeclareParam<int>(
        this, "reroute_ticks", 40,
        "consecutive off-line ticks before a reroute");

    pub_route_ = create_publisher<nuway_msgs::msg::Route>(
        nuway_common::kTopicRoutePlan, nuway_common::qos::Latched());
    pub_reference_line_ = create_publisher<nuway_msgs::msg::ReferenceLine>(
        nuway_common::kTopicReferenceLine, nuway_common::qos::Latched());
    sub_lane_graph_ = create_subscription<nuway_msgs::msg::LaneGraph>(
        nuway_common::kTopicLaneGraph, nuway_common::qos::Latched(),
        [this](const nuway_msgs::msg::LaneGraph& msg) { OnLaneGraph(msg); });
    sub_waypoints_ = create_subscription<nav_msgs::msg::Path>(
        nuway_common::kTopicRouteWaypoints, nuway_common::qos::Latched(),
        [this](const nav_msgs::msg::Path& msg) { OnWaypoints(msg); });
    sub_pose_ = create_subscription<nuway_msgs::msg::EgoState>(
        nuway_common::kTopicPose, nuway_common::qos::Stream(),
        [this](const nuway_msgs::msg::EgoState& msg) { OnPose(msg); });
    sub_reset_ = create_subscription<nuway_msgs::msg::ResetEvent>(
        nuway_common::kTopicResetEvent, nuway_common::qos::Event(),
        [this](const nuway_msgs::msg::ResetEvent& msg) { OnResetEvent(msg); });
  }

 private:
  // Rebuilds the lane graph from the latched message (once per map load).
  void OnLaneGraph(const nuway_msgs::msg::LaneGraph& msg) {
    graph_ = std::make_unique<nuway_map::LaneGraph>(
        nuway_map::LaneGraph::FromMsg(msg));
    RCLCPP_INFO(get_logger(), "lane graph: %zu lanes", graph_->lanes().size());
  }

  // Stamped before the current episode's first tick (docs/02 §7), as tick
  // indices (raw stamps of one tick can differ by nanoseconds between
  // publishers). A zero stamp is "unknown", not "before": the harness may
  // publish unstamped waypoints, which are accepted.
  bool BeforeEpisode(const builtin_interfaces::msg::Time& stamp) const {
    return episode_tick_.has_value() &&
           nuway_common::TickIndex(stamp) < *episode_tick_ &&
           rclcpp::Time(stamp).nanoseconds() != 0;
  }

  // Starts a new episode: records its id and first tick, drops the route
  // state and any waypoints that belong to an earlier episode.
  void OnResetEvent(const nuway_msgs::msg::ResetEvent& msg) {
    // The event topic is transient_local: replays of earlier episodes arrive
    // in no guaranteed order and must not clear anything.
    if (episode_id_.has_value() && msg.episode_id <= *episode_id_) {
      return;
    }
    episode_id_ = msg.episode_id;
    episode_tick_ = nuway_common::TickIndex(msg.header.stamp);
    // Waypoints already stamped for this episode survive: when the Path and
    // the ResetEvent are ready in the same executor wake, the subscriptions
    // are served in creation order (waypoints first), and clearing them here
    // would leave the whole episode with "no waypoints".
    if (!waypoints_.empty() && BeforeEpisode(waypoints_stamp_)) {
      waypoints_.clear();
    }
    ClearRoute();
    RCLCPP_INFO(get_logger(), "reset: episode %u", msg.episode_id);
  }

  // Takes the episode's route (map frame positions; headings unused) and
  // drops the current plan so the next pose plans afresh.
  void OnWaypoints(const nav_msgs::msg::Path& msg) {
    if (BeforeEpisode(msg.header.stamp)) {
      RCLCPP_WARN(get_logger(),
                  "waypoints stamped before the current episode; ignored");
      return;
    }
    waypoints_.clear();
    waypoints_.reserve(msg.poses.size());
    for (const geometry_msgs::msg::PoseStamped& pose : msg.poses) {
      waypoints_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    waypoints_stamp_ = msg.header.stamp;
    ClearRoute();
    RCLCPP_INFO(get_logger(), "route: %zu waypoints", waypoints_.size());
  }

  // The per-tick entry point: plans when there is no line yet, otherwise
  // checks for a reroute, and publishes one diag message stamped with the
  // pose either way (a missing input is reported, not silently skipped).
  // An invalid pose is the upstream no-input output and yields nothing.
  void OnPose(const nuway_msgs::msg::EgoState& msg) {
    double cycle_ms = 0.0;
    std::string message;
    DiagStatus status = DiagStatus::kOk;
    {
      const nuway_common::ScopedTimer timer(&cycle_ms);
      if (BeforeEpisode(msg.header.stamp)) {
        return;  // previous episode
      }
      if (!msg.valid) {
        message = "pose invalid";
      } else if (graph_ == nullptr) {
        message = "no lane graph";
        status = DiagStatus::kWarn;
      } else if (waypoints_.empty()) {
        message = "no waypoints";
      } else if (!line_.has_value()) {
        Plan(msg, 0, &message, &status);
      } else {
        CheckReroute(msg, &message, &status);
      }
    }
    diag_.Publish(msg.header.stamp, cycle_ms, 0.0, status, message);
  }

  // Drops everything derived from the waypoints (the waypoints stay).
  void ClearRoute() {
    line_.reset();
    route_.reset();
    waypoint_s_.clear();
    planned_from_ = 0;
    off_line_ticks_ = 0;
  }

  // Plans from the pose through waypoints [first_waypoint, end), builds the
  // reference line and publishes both; on failure reports through
  // `message`/`status` and leaves the previous line (if any) in place.
  // Also records where each remaining waypoint projects onto the new line
  // (waypoint_s_) so a later reroute knows which ones are already passed.
  void Plan(const nuway_msgs::msg::EgoState& msg, std::size_t first_waypoint,
            std::string* message, DiagStatus* status) {
    const nuway_common::SE2 ego = nuway_common::SE2FromMsg(msg.pose);
    const nuway_common::Vector2dList remaining(
        waypoints_.begin() + static_cast<std::ptrdiff_t>(first_waypoint),
        waypoints_.end());
    std::string error;
    const std::optional<RoutePlan> plan =
        PlanRoute(*graph_, ego, remaining, planner_options_, &error);
    if (!plan.has_value()) {
      *message = "plan failed: " + error;
      *status = DiagStatus::kError;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s",
                           message->c_str());
      return;
    }
    std::optional<nuway_msgs::msg::ReferenceLine> line = BuildReferenceLine(
        *graph_, plan->lane_ids, line_options_, plan->start_s_m);
    if (!line.has_value()) {
      *message = "reference line failed";
      *status = DiagStatus::kError;
      return;
    }
    line->header.stamp = msg.header.stamp;
    nuway_msgs::msg::Route route;
    route.header = line->header;
    route.lane_ids = plan->lane_ids;
    route.goal = GoalPose(*plan);
    // Project every waypoint onto the line for the reroute bookkeeping.
    nuway_common::Vector2dList points;
    points.reserve(line->points.size());
    for (const geometry_msgs::msg::Point& p : line->points) {
      points.emplace_back(p.x, p.y);
    }
    frenet_ = nuway_common::ReferenceLine::FromPoints(points);
    // Waypoints before first_waypoint are passed for good; projecting them
    // onto the new line (which may run near them again) would give them a
    // large s and make a later reroute replan through them.
    planned_from_ = first_waypoint;
    waypoint_s_.assign(waypoints_.size(), -1.0);
    for (std::size_t i = first_waypoint; i < waypoints_.size(); ++i) {
      const Eigen::Vector2d& wp = waypoints_[i];
      const std::optional<nuway_common::FrenetPoint> f = frenet_.ToFrenet(
          wp.x(), wp.y(), 2.0 * planner_options_.waypoint_max_dist_m);
      waypoint_s_[i] = f.has_value() ? f->s : -1.0;
    }
    line_ = std::move(line);
    route_ = route;
    off_line_ticks_ = 0;
    pub_route_->publish(*route_);
    pub_reference_line_->publish(*line_);
    *message = "planned " + std::to_string(route.lane_ids.size()) + " lanes, " +
               std::to_string(line_->points.size()) + " points";
    RCLCPP_INFO(get_logger(), "%s (from waypoint %zu)", message->c_str(),
                first_waypoint);
  }

  // Route.goal: the last waypoint (map frame) with the heading of the last
  // planned lane at the waypoint's projection, since waypoints carry none.
  geometry_msgs::msg::Pose GoalPose(const RoutePlan& plan) const {
    geometry_msgs::msg::Pose goal;
    const Eigen::Vector2d& wp = waypoints_.back();
    goal.position.x = wp.x();
    goal.position.y = wp.y();
    const nuway_map::Lane* lane = graph_->lane(plan.lane_ids.back());
    double yaw = 0.0;
    if (lane != nullptr) {
      const nuway_common::ReferenceLine* line =
          graph_->reference_line(lane->id);
      const std::optional<nuway_common::FrenetPoint> f =
          line->ToFrenet(wp.x(), wp.y(), planner_options_.waypoint_max_dist_m);
      yaw = line->HeadingAt(f.has_value() ? f->s : 0.0);
    }
    goal.orientation =
        nuway_common::QuaternionMsg(nuway_common::YawToQuaternion(yaw));
    return goal;
  }

  // Counts consecutive off-line poses (see the file header) and, once
  // reroute_ticks are reached, replans from the pose through the waypoints
  // not yet passed. The projection search radius is generous
  // (4 * reroute_lateral_m) so a failed projection means "far off", not
  // "just outside the threshold". A reroute is reported as a warning even
  // when the replan succeeds.
  void CheckReroute(const nuway_msgs::msg::EgoState& msg, std::string* message,
                    DiagStatus* status) {
    const nuway_common::SE2 ego = nuway_common::SE2FromMsg(msg.pose);
    const std::optional<nuway_common::FrenetPoint> f =
        frenet_.ToFrenet(ego.x, ego.y, 4.0 * reroute_lateral_m_);
    bool off = !f.has_value();
    if (f.has_value()) {
      const double heading_err =
          std::abs(nuway_common::WrapAngle(frenet_.HeadingAt(f->s) - ego.yaw));
      off = std::abs(f->d) > reroute_lateral_m_ ||
            heading_err > reroute_heading_rad_;
    }
    off_line_ticks_ = off ? off_line_ticks_ + 1 : 0;
    if (off_line_ticks_ < reroute_ticks_) {
      *message =
          off ? "off line (" + std::to_string(off_line_ticks_) + ")" : "";
      return;
    }
    // Replan through the waypoints not yet passed: scan forward from the
    // current plan's first waypoint while the waypoint's s on the line is
    // known and behind the ego's, keeping at least the last one.
    const double ego_s = f.has_value() ? f->s : 0.0;
    std::size_t first = planned_from_;
    while (first + 1 < waypoints_.size() && waypoint_s_[first] >= 0.0 &&
           waypoint_s_[first] < ego_s) {
      ++first;
    }
    RCLCPP_WARN(get_logger(), "rerouting from waypoint %zu", first);
    Plan(msg, first, message, status);
    *message = "reroute: " + *message;
    if (*status == DiagStatus::kOk) {
      *status = DiagStatus::kWarn;
    }
  }

  RoutePlannerOptions planner_options_;
  ReferenceLineOptions line_options_;
  double reroute_lateral_m_ = 5.0;
  double reroute_heading_rad_ = 0.5236;
  int reroute_ticks_ = 40;

  std::unique_ptr<nuway_map::LaneGraph> graph_;
  std::optional<std::uint32_t> episode_id_;
  std::optional<std::int64_t> episode_tick_;  // first tick of the episode
  nuway_common::Vector2dList waypoints_;      // map frame, whole route
  builtin_interfaces::msg::Time waypoints_stamp_;
  std::optional<nuway_msgs::msg::ReferenceLine> line_;  // current line
  std::optional<nuway_msgs::msg::Route> route_;
  nuway_common::ReferenceLine frenet_;  // line_ as a Frenet frame
  std::vector<double> waypoint_s_;      // s of each waypoint on line_, -1 n/a
  std::size_t planned_from_ = 0;        // first waypoint of the current plan
  int off_line_ticks_ = 0;              // consecutive off-line poses

  nuway_common::DiagPublisher diag_;
  rclcpp::Publisher<nuway_msgs::msg::Route>::SharedPtr pub_route_;
  rclcpp::Publisher<nuway_msgs::msg::ReferenceLine>::SharedPtr
      pub_reference_line_;
  rclcpp::Subscription<nuway_msgs::msg::LaneGraph>::SharedPtr sub_lane_graph_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_waypoints_;
  rclcpp::Subscription<nuway_msgs::msg::EgoState>::SharedPtr sub_pose_;
  rclcpp::Subscription<nuway_msgs::msg::ResetEvent>::SharedPtr sub_reset_;
};

}  // namespace
}  // namespace nuway_route

int main(int argc, char** argv) {
  return nuway_common::RunNode<nuway_route::RoutePlannerNode>(argc, argv);
}
