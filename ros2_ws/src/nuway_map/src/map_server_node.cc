// map_server_node (M0 §2.4): waits for <map_dir>/<town>/map.xodr, which
// world_manager writes after connecting to CARLA, builds the LaneGraph and
// publishes it latched on /nuway/map/lane_graph. Serves /nuway/map/nearest_lane
// for tools and tests. The 1 s wall-clock poll is the one timer in the stack:
// it runs at start-up only, before any tick exists, so it cannot affect results
// (docs/02_interfaces.md §2).
//
// "Latched" is the transient_local + reliable QoS profile
// (docs/02_interfaces.md §3.11): the publisher keeps its last message and
// hands it to every subscriber that joins later, so the graph is published
// exactly once and nodes starting in any order still receive it. Stateless
// across episodes: the graph depends only on the town, so the node has
// nothing to clear on /nuway/sim/reset_event and does not subscribe to it.
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <nuway_common/diag.h>
#include <nuway_common/frames.h>
#include <nuway_common/node_main.h>
#include <nuway_common/params.h>
#include <nuway_common/qos.h>
#include <nuway_msgs/msg/lane_graph.hpp>
#include <nuway_msgs/srv/nearest_lane.hpp>

#include "nuway_map/lane_graph.h"
#include "nuway_map/names.h"
#include "nuway_map/opendrive_parser.h"

namespace nuway_map {
namespace {

// Declares the parameters, creates the latched publisher and the service,
// and starts polling for the map file.
class MapServerNode final : public rclcpp::Node {
 public:
  MapServerNode() : rclcpp::Node(kNodeName), diag_(this) {
    map_dir_ = nuway_common::DeclareParam<std::string>(
        this, "map_dir", "data/maps", "directory holding <town>/map.xodr");
    town_ = nuway_common::DeclareParam<std::string>(this, "town", "Town03",
                                                    "CARLA town name");
    options_.default_speed_limit_mps = nuway_common::DeclareParam<double>(
        this, "default_speed_limit_mps", 8.33,
        "speed limit for roads without a <speed> record");
    options_.centerline_spacing_m = nuway_common::DeclareParam<double>(
        this, "centerline_spacing_m", 1.0, "lane centerline sample spacing");
    const auto poll_period_s = nuway_common::DeclareParam<double>(
        this, "poll_period_s", 1.0, "start-up poll interval for map.xodr");

    pub_lane_graph_ = create_publisher<nuway_msgs::msg::LaneGraph>(
        nuway_common::kTopicLaneGraph, nuway_common::qos::Latched());
    srv_nearest_lane_ = create_service<nuway_msgs::srv::NearestLane>(
        kServiceNearestLane,
        [this](
            const std::shared_ptr<nuway_msgs::srv::NearestLane::Request>& req,
            const std::shared_ptr<nuway_msgs::srv::NearestLane::Response>&
                res) { OnNearestLane(*req, res.get()); });
    poll_timer_ = create_wall_timer(
        std::chrono::duration<double>(poll_period_s), [this]() { Poll(); });
    Poll();
  }

 private:
  // <map_dir>/<town>/map.xodr.
  std::string MapPath() const {
    return (std::filesystem::path(map_dir_) / town_ / "map.xodr").string();
  }

  // One poll: if the file exists and parses, build the graph, publish it
  // once, report the build time on diag and cancel the timer; otherwise
  // (absent, empty, or half-written) return and try again next period.
  void Poll() {
    if (graph_ != nullptr) {
      poll_timer_->cancel();
      return;
    }
    const std::string path = MapPath();
    std::error_code ec;
    if (!std::filesystem::exists(path, ec) ||
        std::filesystem::file_size(path, ec) == 0) {
      if (!waiting_logged_) {
        RCLCPP_INFO(get_logger(), "waiting for %s", path.c_str());
        waiting_logged_ = true;
      }
      return;
    }
    std::string error;
    const auto map = LoadOpenDrive(path, &error);
    if (!map.has_value()) {
      // world_manager may still be writing; retry on the next poll. The
      // throttle runs on a steady clock: the node clock is sim time, which
      // stays at 0 until the first tick, so it would log once and go silent.
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000, "%s: %s",
                           path.c_str(), error.c_str());
      return;
    }
    double build_ms = 0.0;
    {
      const nuway_common::ScopedTimer timer(&build_ms);
      graph_ = std::make_unique<LaneGraph>(LaneGraph::Build(*map, options_));
    }
    nuway_msgs::msg::LaneGraph msg = graph_->ToMsg();
    msg.header.stamp = now();
    pub_lane_graph_->publish(msg);
    RCLCPP_INFO(get_logger(),
                "%s: %zu lanes, %zu traffic lights, %zu stop signs, %zu "
                "crosswalks, georeference %s (%.0f ms)",
                path.c_str(), graph_->lanes().size(),
                graph_->traffic_lights().size(), graph_->stop_signs().size(),
                graph_->crosswalks().size(),
                graph_->geo_reference().valid ? "yes" : "no", build_ms);
    diag_.Publish(msg.header.stamp, build_ms, 0.0,
                  nuway_common::DiagStatus::kOk, "lane graph published");
    poll_timer_->cancel();
  }

  // Service callback: LaneGraph::NearestLane on the built graph. found is
  // false before the graph exists or when no drivable lane is within
  // max_dist with a consistent heading.
  void OnNearestLane(const nuway_msgs::srv::NearestLane::Request& req,
                     nuway_msgs::srv::NearestLane::Response* res) {
    res->found = false;
    if (graph_ == nullptr) {
      return;
    }
    const std::optional<LaneQuery> query =
        graph_->NearestLane(req.x, req.y, req.yaw, req.max_dist);
    if (!query.has_value()) {
      return;
    }
    res->found = true;
    res->lane_id = query->lane_id;
    res->s = query->s;
    res->d = query->d;
  }

  std::string map_dir_;
  std::string town_;
  LaneGraphOptions options_;
  bool waiting_logged_ = false;
  std::unique_ptr<LaneGraph> graph_;
  nuway_common::DiagPublisher diag_;
  rclcpp::Publisher<nuway_msgs::msg::LaneGraph>::SharedPtr pub_lane_graph_;
  rclcpp::Service<nuway_msgs::srv::NearestLane>::SharedPtr srv_nearest_lane_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};  // log throttling only
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

}  // namespace
}  // namespace nuway_map

int main(int argc, char** argv) {
  return nuway_common::RunNode<nuway_map::MapServerNode>(argc, argv);
}
