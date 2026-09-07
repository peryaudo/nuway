// Parameter declaration helpers: declare with an explicit default and a
// one-line description, read once at startup (docs/03 §4). Introduced in M0.
#ifndef NUWAY_COMMON_PARAMS_HPP_
#define NUWAY_COMMON_PARAMS_HPP_

#include <string>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace nuway_common {

// Declares `name` with `default_value` and returns its resolved value.
template <typename T>
T DeclareParam(rclcpp::Node* node, const std::string& name,
               const T& default_value, const std::string& description) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  return node->declare_parameter<T>(name, default_value, descriptor);
}

}  // namespace nuway_common

#endif  // NUWAY_COMMON_PARAMS_HPP_
