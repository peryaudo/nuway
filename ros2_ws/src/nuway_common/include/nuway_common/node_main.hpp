// The one main() body of every C++ node: init, spin, shutdown, with the
// node boundary catching what rclcpp may throw (docs/03 §2: no exceptions in
// project code, third-party throws are caught at the node boundary).
// Introduced in M0.
#ifndef NUWAY_COMMON_NODE_MAIN_HPP_
#define NUWAY_COMMON_NODE_MAIN_HPP_

#include <cstdio>
#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace nuway_common {

// Runs NodeT (default-constructed) to completion; returns the exit code.
template <typename NodeT>
int RunNode(int argc, char** argv) {
  try {
    rclcpp::init(argc, argv);
    {
      const std::shared_ptr<NodeT> node = std::make_shared<NodeT>();
      rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "node failed: %s\n", e.what());
    return 1;
  }
}

}  // namespace nuway_common

#endif  // NUWAY_COMMON_NODE_MAIN_HPP_
