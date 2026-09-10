// Safety layer (M1 §3.7): the independent last check on the planner's
// trajectory, run every tick. It re-times the trajectory to the current
// tick (a control-only tick sees the planning tick's trajectory 0.05 s
// later, and every output has t = 0 at its own stamp), replaces a
// degraded planner with the gentlest stop along the last safe path that
// its own collision check clears (then holds it for the episode),
// re-runs the collision check with doubled margins over the first
// seconds and swaps in a max-decel stop along the same path on a hit,
// clips acceleration and curvature to the vehicle limits and
// re-integrates, and checks the ego footprint against the occupancy
// grid's `occupied` channel. No rclcpp; nuway_py binds it in M6.
#ifndef NUWAY_PLANNING_SAFETY_LAYER_H_
#define NUWAY_PLANNING_SAFETY_LAYER_H_

#include <optional>
#include <string>
#include <vector>

#include <nuway_common/agents.h>
#include <nuway_common/geometry.h>
#include <nuway_common/occupancy.h>
#include <nuway_common/trajectory.h>
#include <nuway_control/vehicle_model.h>

#include "nuway_planning/collision_checker.h"

namespace nuway_planning {

struct SafetyOptions {
  // The planner's checker with twice its margins (§3.7 step 2).
  CollisionOptions collision;
  double collision_horizon_s = 3.0;
  double occupancy_threshold = 0.6;  // `occupied` above this: a hit
  double occupancy_horizon_s = 3.0;
  double footprint_sample_m = 0.5;  // occupancy sample spacing
  double a_gentle_mps2 = 1.5;       // the degraded planner's first choice
  double a_min_mps2 = -6.0;         // max decel (negative)
  double a_max_mps2 = 3.0;
  double kappa_phys = 0.96;

  static SafetyOptions FromVehicleModel(const nuway_control::VehicleModel& m);
};

// The `occupied` channel of an OccupancyGridMC with the base_link pose it
// was expressed in (the ego pose at the grid's stamp, docs/02 §1).
struct OccupancyView {
  nuway_common::GridSpec spec;
  std::vector<float> occupied;  // [height][width], row-major
  nuway_common::SE2 base_pose;  // map -> base_link at the grid's stamp
};

struct SafetyInput {
  nuway_common::SE2 ego_pose;  // this tick's base_link pose, map frame
  double ego_speed_mps = 0.0;
  // The planner's trajectory (t relative to its own stamp), its source
  // and how long ago it was stamped; null when the planner is degraded.
  const nuway_common::Trajectory* trajectory = nullptr;
  std::string source;
  double trajectory_age_s = 0.0;
  bool planner_degraded = false;
  const std::vector<nuway_common::AgentState>* agents = nullptr;
  const nuway_common::PredictionSet* predictions = nullptr;
  const OccupancyView* occupancy = nullptr;
};

struct SafetyOutput {
  nuway_common::Trajectory trajectory;  // 81 points, t = 0 at this tick
  std::string source;  // the input's, or "fallback" for a replaced profile
  bool intervened = false;
  std::string reason;  // "collision", "occupancy", "limits", "degraded"
};

// The trajectory re-timed by `age_s`: point i is the input at t_i + age,
// the end extended at constant speed along its heading (§3.7).
nuway_common::Trajectory Retime(const nuway_common::Trajectory& traj,
                                double age_s);

// A constant-deceleration stop from v0 along the path of `path` (its
// points as the geometry, arc length as the parameter), 81 points.
nuway_common::Trajectory StopAlongPath(const nuway_common::Trajectory& path,
                                       double v0_mps, double decel_mps2);

class SafetyLayer {
 public:
  explicit SafetyLayer(SafetyOptions options);

  SafetyOutput Step(const SafetyInput& in);

  // Drops the last safe trajectory and any held fallback (ResetEvent).
  void Reset();

  const SafetyOptions& options() const { return options_; }

 private:
  bool Collides(const nuway_common::Trajectory& traj,
                const SafetyInput& in) const;
  bool OccupancyHit(const nuway_common::Trajectory& traj,
                    const OccupancyView& grid) const;
  // Clips a and kappa; re-integrates the kinematics when anything moved.
  bool EnforceLimits(nuway_common::Trajectory* traj) const;

  SafetyOptions options_;
  CollisionChecker checker_;
  std::optional<nuway_common::Trajectory> last_safe_;
  std::optional<nuway_common::Trajectory> held_;  // the degraded fallback
};

}  // namespace nuway_planning

#endif  // NUWAY_PLANNING_SAFETY_LAYER_H_
