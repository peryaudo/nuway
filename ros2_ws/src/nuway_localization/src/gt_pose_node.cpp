// gt_pose_node (M0 §2.8): the localization cheat twin. Every tick it turns
// /nuway/gt/ego_odom + /nuway/sim/vehicle_state (steering angle) into
// /nuway/loc/pose (EgoState, valid: true) and the odom->base_link TF;
// map->odom is the identity, published on planning ticks like the M5
// smoother. ax/ay are the one-tick finite difference of the body velocity.
//
// Current-tick barrier over the two inputs (docs/02 §2); on TickTimeout(k)
// the missing input is degraded until the next ResetEvent: without the
// odometry the node publishes valid: false (no-input convention), without the
// vehicle state it publishes steering_angle 0. Optional noise.* parameters
// (translation/yaw sigma, latency in ticks) stress downstream before M5;
// the RNG is seeded from noise.seed plus the episode id at every reset, so a
// noisy run is still reproducible. Cross-tick state (barrier, buffers, the
// previous velocity, the RNG) is dropped on ResetEvent; messages stamped
// before the reset are ignored (docs/02 §7).
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <nuway_common/diag.hpp>
#include <nuway_common/frames.hpp>
#include <nuway_common/geometry.hpp>
#include <nuway_common/node_main.hpp>
#include <nuway_common/params.hpp>
#include <nuway_common/qos.hpp>
#include <nuway_common/ros_conv.hpp>
#include <nuway_common/tick.hpp>
#include <nuway_msgs/msg/ego_state.hpp>
#include <nuway_msgs/msg/reset_event.hpp>
#include <nuway_msgs/msg/tick_timeout.hpp>
#include <nuway_msgs/msg/vehicle_state.hpp>

#include "nuway_localization/names.hpp"

namespace nuway_localization {
namespace {

using nuway_common::DiagStatus;

constexpr const char* kInputOdom = "ego_odom";
constexpr const char* kInputVehicleState = "vehicle_state";
// Buffered inputs older than this many ticks behind the newest are dropped.
constexpr std::int64_t kBufferTicks = 4;
// Reseeded from noise.seed at start-up and at every reset.
constexpr std::uint64_t kDefaultSeed = 0;

class GtPoseNode final : public rclcpp::Node {
 public:
  GtPoseNode()
      : rclcpp::Node(kGtPoseNodeName),
        // Deterministic by design (docs/02 §2); reseeded per episode.
        rng_(kDefaultSeed),  // NOLINT(bugprone-random-generator-seed)
        barrier_({kInputOdom, kInputVehicleState}),
        diag_(this),
        tf_broadcaster_(*this) {
    translation_sigma_m_ = nuway_common::DeclareParam<double>(
        this, "noise.translation_sigma_m", 0.0, "Gaussian noise on x, y");
    yaw_sigma_rad_ = nuway_common::DeclareParam<double>(
        this, "noise.yaw_sigma_rad", 0.0, "Gaussian noise on yaw");
    latency_ticks_ = nuway_common::DeclareParam<int>(
        this, "noise.latency_ticks", 0, "publish the pose of tick k - latency");
    seed_ = nuway_common::DeclareParam<int>(this, "noise.seed", 0,
                                            "RNG seed (+ episode id)");
    rng_.seed(static_cast<std::uint64_t>(seed_));

    pub_pose_ = create_publisher<nuway_msgs::msg::EgoState>(
        nuway_common::kTopicPose, nuway_common::qos::Stream());
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        nuway_common::kTopicGtEgoOdom, nuway_common::qos::Stream(),
        [this](const nav_msgs::msg::Odometry& msg) { OnOdom(msg); });
    sub_vehicle_state_ = create_subscription<nuway_msgs::msg::VehicleState>(
        nuway_common::kTopicVehicleState, nuway_common::qos::Stream(),
        [this](const nuway_msgs::msg::VehicleState& msg) {
          OnVehicleState(msg);
        });
    sub_reset_ = create_subscription<nuway_msgs::msg::ResetEvent>(
        nuway_common::kTopicResetEvent, nuway_common::qos::Event(),
        [this](const nuway_msgs::msg::ResetEvent& msg) { OnResetEvent(msg); });
    sub_tick_timeout_ = create_subscription<nuway_msgs::msg::TickTimeout>(
        nuway_common::kTopicTickTimeout, nuway_common::qos::Event(),
        [this](const nuway_msgs::msg::TickTimeout& msg) {
          OnTickTimeout(msg);
        });
  }

 private:
  template <typename MsgT>
  static void Prune(std::map<std::int64_t, MsgT>* buffer, std::int64_t k) {
    while (!buffer->empty() && buffer->begin()->first < k - kBufferTicks) {
      buffer->erase(buffer->begin());
    }
  }

  void OnResetEvent(const nuway_msgs::msg::ResetEvent& msg) {
    episode_start_k_ = nuway_common::TickIndex(msg.header.stamp);
    last_published_k_ = episode_start_k_ - 1;
    barrier_.Reset();
    odoms_.clear();
    vehicle_states_.clear();
    history_.clear();
    prev_velocity_.reset();
    rng_.seed(static_cast<std::uint64_t>(seed_) + msg.episode_id);
    RCLCPP_INFO(get_logger(), "reset: episode %u", msg.episode_id);
  }

  void OnOdom(const nav_msgs::msg::Odometry& msg) {
    const std::int64_t k = nuway_common::TickIndex(msg.header.stamp);
    if (k < episode_start_k_) {
      return;
    }
    odoms_[k] = msg;
    Prune(&odoms_, k);
    barrier_.Arrive(kInputOdom, k);
    if (barrier_.IsComplete(k)) {
      Run(k);
    }
  }

  void OnVehicleState(const nuway_msgs::msg::VehicleState& msg) {
    const std::int64_t k = nuway_common::TickIndex(msg.header.stamp);
    if (k < episode_start_k_) {
      return;
    }
    vehicle_states_[k] = msg;
    Prune(&vehicle_states_, k);
    barrier_.Arrive(kInputVehicleState, k);
    if (barrier_.IsComplete(k)) {
      Run(k);
    }
  }

  void OnTickTimeout(const nuway_msgs::msg::TickTimeout& msg) {
    const std::int64_t k = nuway_common::TickIndex(msg.header.stamp);
    if (k < episode_start_k_) {
      return;
    }
    for (const std::string& name : barrier_.Missing(k)) {
      barrier_.Degrade(name);
      RCLCPP_WARN(get_logger(), "tick %ld timed out; %s degraded until reset",
                  static_cast<long>(k), name.c_str());
    }
    Run(k);
  }

  // The GT odometry to publish for tick k: the one of tick k - latency, or
  // the oldest buffered one right after a reset.
  const nav_msgs::msg::Odometry* Delayed(std::int64_t k) const {
    if (history_.empty()) {
      return nullptr;
    }
    const std::int64_t wanted = k - latency_ticks_;
    for (const auto& [tick, odom] : history_) {
      if (tick >= wanted) {
        return &odom;
      }
    }
    return &history_.back().second;
  }

  void Run(std::int64_t k) {
    if (k <= last_published_k_) {
      return;
    }
    last_published_k_ = k;
    double cycle_ms = 0.0;
    nuway_msgs::msg::EgoState out;
    DiagStatus status = DiagStatus::kOk;
    std::string message;
    {
      const nuway_common::ScopedTimer timer(&cycle_ms);
      const auto odom_it = odoms_.find(k);
      if (odom_it == odoms_.end()) {
        out = last_pose_;
        out.header.stamp = nuway_common::TickStamp(k);
        out.header.frame_id = nuway_common::kFrameMap;
        out.valid = false;
        status = DiagStatus::kWarn;
        message = "no ego_odom for the tick (degraded)";
      } else {
        history_.emplace_back(k, odom_it->second);
        while (history_.size() > static_cast<std::size_t>(latency_ticks_) + 1) {
          history_.pop_front();
        }
        const nav_msgs::msg::Odometry& odom = *Delayed(k);
        out = Build(k, odom_it->second, odom);
        const auto vs_it = vehicle_states_.find(k);
        if (vs_it != vehicle_states_.end()) {
          out.steering_angle = vs_it->second.steering_angle;
        } else {
          status = DiagStatus::kWarn;
          message = "no vehicle_state for the tick (degraded)";
        }
        last_pose_ = out;
        PublishTf(out);
      }
    }
    pub_pose_->publish(out);
    diag_.Publish(out.header.stamp, cycle_ms, 0.0, status, message);
  }

  // EgoState of tick k from the current odometry (velocities, accelerations)
  // and the possibly delayed one (pose), with noise applied.
  nuway_msgs::msg::EgoState Build(std::int64_t k,
                                  const nav_msgs::msg::Odometry& current,
                                  const nav_msgs::msg::Odometry& delayed) {
    nuway_msgs::msg::EgoState out;
    out.header.stamp = current.header.stamp;
    out.header.frame_id = nuway_common::kFrameMap;
    nuway_common::SE3 pose = nuway_common::SE3FromMsg(delayed.pose.pose);
    if (translation_sigma_m_ > 0.0 || yaw_sigma_rad_ > 0.0) {
      std::normal_distribution<double> unit(0.0, 1.0);
      pose.translation.x() += translation_sigma_m_ * unit(rng_);
      pose.translation.y() += translation_sigma_m_ * unit(rng_);
      const double yaw_noise = yaw_sigma_rad_ * unit(rng_);
      pose.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(
                          yaw_noise, Eigen::Vector3d::UnitZ())) *
                      pose.rotation;
      out.covariance[0] = translation_sigma_m_ * translation_sigma_m_;
      out.covariance[7] = translation_sigma_m_ * translation_sigma_m_;
      out.covariance[35] = yaw_sigma_rad_ * yaw_sigma_rad_;
    }
    out.pose = nuway_common::PoseMsg(pose);
    const Eigen::Vector2d velocity(current.twist.twist.linear.x,
                                   current.twist.twist.linear.y);
    out.vx = velocity.x();
    out.vy = velocity.y();
    out.yaw_rate = current.twist.twist.angular.z;
    if (prev_velocity_.has_value()) {
      const double dt_s = static_cast<double>(k - prev_velocity_->first) *
                          nuway_common::kTickDtS;
      const Eigen::Vector2d accel =
          (velocity - prev_velocity_->second) / (dt_s > 0.0 ? dt_s : 1.0);
      out.ax = accel.x();
      out.ay = accel.y();
    }
    prev_velocity_ = std::make_pair(k, velocity);
    out.valid = true;
    return out;
  }

  void PublishTf(const nuway_msgs::msg::EgoState& state) {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    geometry_msgs::msg::TransformStamped base;
    base.header.stamp = state.header.stamp;
    base.header.frame_id = nuway_common::kFrameOdom;
    base.child_frame_id = nuway_common::kFrameBaseLink;
    base.transform.translation.x = state.pose.position.x;
    base.transform.translation.y = state.pose.position.y;
    base.transform.translation.z = state.pose.position.z;
    base.transform.rotation = state.pose.orientation;
    transforms.push_back(base);
    if (nuway_common::IsPlanningTick(
            nuway_common::TickIndex(state.header.stamp))) {
      geometry_msgs::msg::TransformStamped odom;
      odom.header.stamp = state.header.stamp;
      odom.header.frame_id = nuway_common::kFrameMap;
      odom.child_frame_id = nuway_common::kFrameOdom;
      odom.transform.rotation.w = 1.0;
      transforms.push_back(odom);
    }
    tf_broadcaster_.sendTransform(transforms);
  }

  double translation_sigma_m_ = 0.0;
  double yaw_sigma_rad_ = 0.0;
  int latency_ticks_ = 0;
  int seed_ = 0;
  std::mt19937_64 rng_;

  nuway_common::TickBarrier barrier_;
  std::int64_t episode_start_k_ = 0;
  std::int64_t last_published_k_ = -1;
  std::map<std::int64_t, nav_msgs::msg::Odometry> odoms_;
  std::map<std::int64_t, nuway_msgs::msg::VehicleState> vehicle_states_;
  std::deque<std::pair<std::int64_t, nav_msgs::msg::Odometry>> history_;
  std::optional<std::pair<std::int64_t, Eigen::Vector2d>> prev_velocity_;
  nuway_msgs::msg::EgoState last_pose_;

  nuway_common::DiagPublisher diag_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Publisher<nuway_msgs::msg::EgoState>::SharedPtr pub_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<nuway_msgs::msg::VehicleState>::SharedPtr
      sub_vehicle_state_;
  rclcpp::Subscription<nuway_msgs::msg::ResetEvent>::SharedPtr sub_reset_;
  rclcpp::Subscription<nuway_msgs::msg::TickTimeout>::SharedPtr
      sub_tick_timeout_;
};

}  // namespace
}  // namespace nuway_localization

int main(int argc, char** argv) {
  return nuway_common::RunNode<nuway_localization::GtPoseNode>(argc, argv);
}
