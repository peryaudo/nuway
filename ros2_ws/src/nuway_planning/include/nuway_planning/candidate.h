// One planner candidate (M1 §3.3-§3.6): the 81-point map-frame trajectory
// the safety layer and the controller consume, plus the Frenet samples it
// was built from (the selector's lateral terms and the QP's references read
// those), the lattice parameters that produced it and the bookkeeping the
// filter, the QP and the selector add. No rclcpp; the node converts to
// nuway_msgs/Trajectory and TrajectoryCandidates.
#ifndef NUWAY_PLANNING_CANDIDATE_H_
#define NUWAY_PLANNING_CANDIDATE_H_

#include <cstdint>
#include <string>
#include <vector>

#include <nuway_common/frenet.h>
#include <nuway_common/trajectory.h>

namespace nuway_planning {

// Which longitudinal profile a candidate carries; for the viz table and the
// tests, not consulted by the selector.
enum class SpeedKind : std::uint8_t {
  kKeep,        // quartic to a target speed (FREE / FOLLOW, the YIELD "go")
  kGap,         // quintic to the lead's gap position (FOLLOW)
  kStop,        // quintic to rest at the decision's stop_s (STOP / YIELD)
  kHardStop,    // constant a_min to rest (STOP, and the injected hard stop)
  kGentleStop,  // quintic to rest at a_gentle (the injected gentle stop)
};

const char* SpeedKindName(SpeedKind kind);

struct Candidate {
  std::uint32_t id = 0;
  // "lattice" for the sampled candidates, "stop" for the injected pair
  // (docs/02 §4 Trajectory.source).
  std::string source = "lattice";
  // The §3.3 injected pair: exempt from the feasibility filter and from the
  // QP, so the selector's input is never empty.
  bool injected = false;
  nuway_common::Trajectory trajectory;            // 81 points, map frame
  std::vector<nuway_common::FrenetState> frenet;  // one per point
  double d_f_m = 0.0;         // lateral target relative to the line
  double s_f_m = 0.0;         // arc length where the path reaches d_f
  double v_target_mps = 0.0;  // the speed the profile aims at (viz)
  double horizon_s = 8.0;     // the polynomial's own horizon before extension
  SpeedKind speed_kind = SpeedKind::kKeep;
  // Empty when feasible; otherwise the first rule the candidate failed
  // ("kappa", "accel", "a_lat", "bounds", "reverse", "collision").
  std::string reject;
  // Task 6-7 bookkeeping: refined by the QP, refined with slack, total
  // cost and its breakdown in the selector's term order.
  bool refined = false;
  bool qp_relaxed = false;
  double cost = 0.0;
  std::vector<double> cost_breakdown;

  bool feasible() const { return reject.empty(); }
};

}  // namespace nuway_planning

#endif  // NUWAY_PLANNING_CANDIDATE_H_
