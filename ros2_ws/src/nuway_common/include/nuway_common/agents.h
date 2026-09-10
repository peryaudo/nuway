// Plain-struct images of nuway_msgs/Agent and nuway_msgs/PredictionSamples
// for the libraries that reason about other road users without rclcpp: the
// constant-velocity predictor (M1 §3.1), the behavior FSM, the collision
// checker and the safety layer (M1 §3.2, §3.4, §3.7). Nodes convert the
// messages with ros_conv.h and hand these structs to the libraries. All
// poses are in the map frame (ROS convention, x forward, y left, yaw
// counter-clockwise, metres and radians) once they have passed the frame
// rule of M1 §3.1 (AgentsToMap). Introduced in M1.
#ifndef NUWAY_COMMON_AGENTS_H_
#define NUWAY_COMMON_AGENTS_H_

#include <cmath>
#include <cstdint>
#include <vector>

#include <nuway_msgs/msg/agent.hpp>

#include "nuway_common/geometry.h"

namespace nuway_common {

// Agent.class_id values, initialised from the generated message constants so
// the two cannot drift (docs/03 §2.1).
enum class AgentClass : std::uint8_t {
  kUnknown = nuway_msgs::msg::Agent::CLASS_UNKNOWN,
  kCar = nuway_msgs::msg::Agent::CLASS_CAR,
  kTruck = nuway_msgs::msg::Agent::CLASS_TRUCK,
  kBicycle = nuway_msgs::msg::Agent::CLASS_BICYCLE,
  kMotorcycle = nuway_msgs::msg::Agent::CLASS_MOTORCYCLE,
  kPedestrian = nuway_msgs::msg::Agent::CLASS_PEDESTRIAN,
  kStaticObstacle = nuway_msgs::msg::Agent::CLASS_STATIC_OBSTACLE,
};

// True for the classes that drive on lanes (cars, trucks, two-wheelers):
// the ones the constant-velocity predictor may follow a lane with and the
// FSM treats as traffic rather than as pedestrians or furniture.
inline bool IsVehicle(AgentClass cls) {
  return cls == AgentClass::kCar || cls == AgentClass::kTruck ||
         cls == AgentClass::kBicycle || cls == AgentClass::kMotorcycle;
}

// One observed agent: box centre pose, footprint and frame-aligned velocity
// (the velocity is expressed in the same frame as the pose, so a map-frame
// agent moving east has vx > 0 whatever its yaw).
struct AgentState {
  std::uint32_t id = 0;
  AgentClass class_id = AgentClass::kUnknown;
  SE2 pose;  // box centre, map frame
  double length_m = 0.0;
  double width_m = 0.0;
  double vx_mps = 0.0;  // map-frame velocity
  double vy_mps = 0.0;
  double yaw_rate_radps = 0.0;
  bool visible = true;

  double speed_mps() const {
    return std::sqrt((vx_mps * vx_mps) + (vy_mps * vy_mps));
  }
};

// The C++ image of nuway_msgs/PredictionSamples (docs/02 §4): S joint
// samples of A agents over T steps of dt seconds, map frame. xy is
// [S][A][T][2] and yaw [S][A][T], flattened row-major exactly as on the
// wire; PoseAt() indexes them. A step t (0-based) is at time (t + 1) * dt
// after the header stamp: the samples start at dt, the current pose is the
// AgentState of the same tick.
struct PredictionSet {
  std::vector<std::uint32_t> agent_ids;
  int num_samples = 0;
  int num_timesteps = 0;
  double dt_s = 0.5;
  std::vector<double> xy;
  std::vector<double> yaw;
  std::vector<double> sample_weight;

  int num_agents() const { return static_cast<int>(agent_ids.size()); }

  // Pose of agent index a in sample s at step t; the caller keeps the
  // indices in range.
  SE2 PoseAt(int s, int a, int t) const {
    const std::size_t agents = agent_ids.size();
    const auto steps = static_cast<std::size_t>(num_timesteps);
    const std::size_t flat = (((static_cast<std::size_t>(s) * agents) +
                               static_cast<std::size_t>(a)) *
                              steps) +
                             static_cast<std::size_t>(t);
    return SE2{xy[2 * flat], xy[(2 * flat) + 1], yaw[flat]};
  }

  // Index of an agent id in agent_ids, or -1.
  int IndexOf(std::uint32_t id) const {
    for (int a = 0; a < num_agents(); ++a) {
      if (agent_ids[static_cast<std::size_t>(a)] == id) {
        return a;
      }
    }
    return -1;
  }

  // Time of step t relative to the stamp, s.
  double TimeAt(int t) const { return static_cast<double>(t + 1) * dt_s; }
};

}  // namespace nuway_common

#endif  // NUWAY_COMMON_AGENTS_H_
