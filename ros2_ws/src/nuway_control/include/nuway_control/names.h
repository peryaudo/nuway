// Node name constants of nuway_control (M0).
#ifndef NUWAY_CONTROL_NAMES_H_
#define NUWAY_CONTROL_NAMES_H_

namespace nuway_control {

// The M0 controller node; the launch files and the profile's per-node
// parameter block (docs/02 §5) address it by this name.
constexpr const char* kNodeName = "pure_pursuit_pid_node";

}  // namespace nuway_control

#endif  // NUWAY_CONTROL_NAMES_H_
