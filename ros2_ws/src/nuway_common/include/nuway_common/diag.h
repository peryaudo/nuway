// NodeDiag publishing helper and a scoped wall-clock timer for cycle_ms
// (docs/02_interfaces.md §7). Every node owns one DiagPublisher on
// /nuway/diag/<node> and wraps each callback in a ScopedTimer. The timer
// measures processing time for reporting only; nothing in the stack acts on
// wall clock (docs/00 §2.5). Introduced in M0.
#ifndef NUWAY_COMMON_DIAG_H_
#define NUWAY_COMMON_DIAG_H_

#include <chrono>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nuway_msgs/msg/node_diag.hpp>

#include "nuway_common/frames.h"
#include "nuway_common/qos.h"

namespace nuway_common {

// Status values of NodeDiag, initialised from the message constants.
enum class DiagStatus : std::uint8_t {
  kOk = nuway_msgs::msg::NodeDiag::STATUS_OK,
  kWarn = nuway_msgs::msg::NodeDiag::STATUS_WARN,
  kError = nuway_msgs::msg::NodeDiag::STATUS_ERROR,
};

// Measures wall-clock milliseconds from construction to destruction into the
// double it was given.
class ScopedTimer final {
 public:
  explicit ScopedTimer(double* elapsed_ms)
      : elapsed_ms_(elapsed_ms), start_(std::chrono::steady_clock::now()) {}
  ScopedTimer(const ScopedTimer&) = delete;
  ScopedTimer& operator=(const ScopedTimer&) = delete;
  ScopedTimer(ScopedTimer&&) = delete;
  ScopedTimer& operator=(ScopedTimer&&) = delete;
  ~ScopedTimer() {
    const auto end = std::chrono::steady_clock::now();
    *elapsed_ms_ =
        std::chrono::duration<double, std::milli>(end - start_).count();
  }

 private:
  double* elapsed_ms_;
  std::chrono::steady_clock::time_point start_;
};

// Publishes nuway_msgs/NodeDiag on /nuway/diag/<node_name>.
class DiagPublisher final {
 public:
  explicit DiagPublisher(rclcpp::Node* node)
      : node_name_(node->get_name()),
        pub_(node->create_publisher<nuway_msgs::msg::NodeDiag>(
            std::string(kTopicDiagPrefix) + node_name_, qos::Diag())) {}

  void Publish(const builtin_interfaces::msg::Time& stamp, double cycle_ms,
               double input_age_ms, DiagStatus status,
               const std::string& message) {
    nuway_msgs::msg::NodeDiag msg;
    msg.header.stamp = stamp;
    msg.node = node_name_;
    msg.cycle_ms = static_cast<float>(cycle_ms);
    msg.input_age_ms = static_cast<float>(input_age_ms);
    msg.status = static_cast<std::uint8_t>(status);
    msg.message = message;
    pub_->publish(msg);
  }

 private:
  std::string node_name_;
  rclcpp::Publisher<nuway_msgs::msg::NodeDiag>::SharedPtr pub_;
};

}  // namespace nuway_common

#endif  // NUWAY_COMMON_DIAG_H_
