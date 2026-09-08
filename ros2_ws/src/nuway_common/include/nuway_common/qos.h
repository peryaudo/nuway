// The named QoS profiles of docs/02_interfaces.md §3.11. Every publisher and
// subscription in a nuway node names one of these; no inline rclcpp::QoS
// construction anywhere else. Introduced in M0.
#ifndef NUWAY_COMMON_QOS_H_
#define NUWAY_COMMON_QOS_H_

#include <rclcpp/qos.hpp>

namespace nuway_common {
namespace qos {

// CARLA-native sensor topics: best_effort, volatile, keep_last 1.
inline rclcpp::QoS Sensor() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

// Every per-tick /nuway/** data topic: reliable, volatile, keep_last 2.
inline rclcpp::QoS Stream() {
  return rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile();
}

// Latched statics (lane graph, route, reference line, camera_info, tf_static).
inline rclcpp::QoS Latched() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

// Events (reset_event, tick_timeout, route waypoints): reliable, transient
// local, keep_last 10.
inline rclcpp::QoS Event() {
  return rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
}

// /nuway/diag/**: reliable, volatile, keep_last 10.
inline rclcpp::QoS Diag() {
  return rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
}

// /nuway/viz/**: best_effort, volatile, keep_last 1.
inline rclcpp::QoS Viz() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

}  // namespace qos
}  // namespace nuway_common

#endif  // NUWAY_COMMON_QOS_H_
