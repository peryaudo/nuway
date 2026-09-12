// Behavior FSM (M1 §3.2): the rule layer that turns the scene into one
// BehaviorDecision per planning tick for the lattice sampler. Pure library;
// behavior_fsm_node is the ROS shell and nuway_py binds it for the M6
// expert.
//
// What an FSM does here. The lattice can only optimise what it is told to
// aim at; somebody has to decide *which* lane, *whether* to follow the car
// ahead, stop at a line or yield to a crossing car, and at *what* speed.
// Those are discrete choices with hysteresis (a lane change, once begun,
// is kept for a few seconds; a light seen yellow once is answered once),
// which is exactly what a finite-state machine with timers expresses and a
// per-tick cost function does not. The FSM has two independent state
// variables: the lateral intent (keep / change left / change right) and
// the longitudinal mode (free / follow / yield / stop). Every tick it
// re-evaluates both from the inputs and its own timers, so the states are
// "current intent", not a plan.
//
// Rules, in priority order (longitudinal): STOP for a red or dilemma-zone
// yellow light on the route with the stop line ahead, an unhonoured stop
// sign, or the route goal within stopping distance; YIELD when another
// agent is predicted to cross the ego corridor ahead before the ego gets
// there (or a pedestrian to be in it); FOLLOW when a lead vehicle is in
// the lane ahead; FREE otherwise. The target speed is the minimum of the
// posted limit, the curvature speed profile (route_line.h) and, when
// following, the speed the Intelligent Driver Model (Treiber 2000) drives
// toward behind the lead. Lateral: a change is requested when the route
// leaves the current lane for a neighbour, or (overtake_enabled) when the
// lead is slow for long enough and the neighbour lane has a clear gap;
// it commits for change_commit_s and aborts if the gap closes.
//
// Cross-tick state: the lateral commitment and its timer, the slow-lead
// timer, the yellow-light latches, the honoured stop signs and the stop-sign
// dwell timer, the projection hint. Reset() drops all of it (ResetEvent).
#ifndef NUWAY_PLANNING_BEHAVIOR_FSM_H_
#define NUWAY_PLANNING_BEHAVIOR_FSM_H_

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <nuway_common/agents.h>
#include <nuway_msgs/msg/behavior_decision.hpp>

#include "nuway_planning/route_line.h"
#include "nuway_planning/scene.h"

namespace nuway_planning {

// BehaviorDecision.lateral / .longitudinal values, from the message
// constants (docs/03 §2.1).
enum class Lateral : std::uint8_t {
  kKeep = nuway_msgs::msg::BehaviorDecision::LATERAL_KEEP,
  kChangeLeft = nuway_msgs::msg::BehaviorDecision::LATERAL_CHANGE_LEFT,
  kChangeRight = nuway_msgs::msg::BehaviorDecision::LATERAL_CHANGE_RIGHT,
};

enum class Longitudinal : std::uint8_t {
  kFree = nuway_msgs::msg::BehaviorDecision::LONGITUDINAL_FREE,
  kFollow = nuway_msgs::msg::BehaviorDecision::LONGITUDINAL_FOLLOW,
  kYield = nuway_msgs::msg::BehaviorDecision::LONGITUDINAL_YIELD,
  kStop = nuway_msgs::msg::BehaviorDecision::LONGITUDINAL_STOP,
};

// Tuning knobs (M1 §3.2); mirrored by config/defaults.yaml.
struct BehaviorFsmOptions {
  // Lateral.
  bool overtake_enabled = false;      // request a change to pass a slow lead
  double slow_lead_margin_mps = 3.0;  // lead slower than limit - this ...
  double slow_lead_s = 4.0;           // ... for this long triggers an overtake
  double gap_back_m = 15.0;  // target-lane gap: no agent within [-back, +ahead]
  double gap_ahead_m = 25.0;
  double gap_horizon_s = 3.0;    // over the predictions up to this time
  double change_commit_s = 3.0;  // a change is kept at least this long
  double abort_gap_m = 8.0;  // ... unless a target-lane agent gets this close
  // Following.
  double follow_range_m = 60.0;  // lead search ahead
  double lane_slack_m = 0.5;     // |d - d_lane| <= half width + slack
  double idm_s0_m = 2.0;         // IDM jam distance
  double idm_t_s = 1.5;          // IDM time headway
  double idm_a_mps2 = 1.5;       // IDM max acceleration
  double idm_b_mps2 = 2.5;       // IDM comfortable deceleration
  double idm_horizon_s = 2.0;    // target = v + a_idm * horizon
  // Stopping.
  double a_comf_mps2 = 2.5;  // comfortable braking (stopping distance, yellow)
  double t_react_s = 0.3;    // reaction time in the dilemma-zone rule
  double stop_lookahead_m = 5.0;  // stopping distance = v^2 / (2 a_comf) + this
  double stop_margin_m = 1.0;     // stop_s = stop line - this
  double passed_line_m = 1.0;     // a line this far behind the ego is passed
  double unknown_confidence = 0.5;   // below: the light state is unknown
  double stop_sign_speed_mps = 0.2;  // honoured: slower than this ...
  double stop_sign_dist_m = 3.0;     // ... within this of the line ...
  double stop_sign_hold_s = 1.0;     // ... for this long
  // Yielding.
  double yield_horizon_s = 4.0;      // predicted crossings up to this time
  double yield_lookahead_m = 60.0;   // ... this far ahead on the route
  double yield_margin_m = 0.5;       // corridor half width slack
  double yield_time_margin_s = 1.5;  // agent must be clear this long before us
  double yield_min_speed_mps = 0.3;  // slower agents are not crossing (the
                                     // collision check handles a standing one)
  double crossing_angle_rad = 0.5;   // heading off the line by more: crossing
  double yield_stop_back_m = 3.0;    // stop_s = s_conflict - this
  double yield_hold_s = 1.0;  // a yield is kept at least this long (hysteresis)
  // Speed profile and projection.
  SpeedProfileOptions speed;
  double projection_max_dist_m = 5.0;
  double projection_back_m = 10.0;
  double projection_ahead_m = 50.0;
  double lane_search_dist_m = 3.0;  // NearestLane radius for the ego lane
  double default_lane_half_width_m = 1.75;
  double ego_front_m = 3.9;  // rear axle to front bumper (gap to the lead)
};

// One tick's decision (the fields of nuway_msgs/BehaviorDecision).
struct BehaviorOutput {
  Lateral lateral = Lateral::kKeep;
  std::uint32_t target_lane_id = 0;
  Longitudinal longitudinal = Longitudinal::kStop;
  std::uint32_t lead_agent_id = 0;
  double stop_s = -1.0;  // on the route line; -1 none
  double target_speed_mps = 0.0;
  std::string reason = "no_input";
  // Not on the wire: the ego's projection, for the node's diagnostics.
  double ego_s = 0.0;
  double ego_d = 0.0;
};

// The no-input decision of docs/02 §2.
BehaviorOutput NoInputDecision();

class BehaviorFsm {
 public:
  explicit BehaviorFsm(BehaviorFsmOptions options);

  // Drops every cross-tick state (ResetEvent, docs/02 §7).
  void Reset();

  // One planning tick. A missing route or an ego off the line yields the
  // no-input decision.
  BehaviorOutput Step(const SceneInput& in);

  const BehaviorFsmOptions& options() const { return options_; }

 private:
  // Ego lane from the graph (NearestLane) or, without a graph, the route
  // lane at s.
  std::uint32_t EgoLane(const SceneInput& in, double s) const;
  // Lane half width at s of the given lane (graph width, else the default).
  double LaneHalfWidth(const SceneInput& in, std::uint32_t lane_id, double x,
                       double y) const;
  // The nearest agent ahead within follow_range in the lane band around
  // lateral offset d_lane; nullptr when none.
  const nuway_common::AgentState* FindLead(const SceneInput& in, double s_ego,
                                           double d_lane,
                                           double half_width) const;
  // Lateral intent this tick; updates the change timers.
  void StepLateral(const SceneInput& in, double s_ego, double d_ego,
                   std::uint32_t ego_lane, const nuway_common::AgentState* lead,
                   double v_limit, BehaviorOutput* out);
  // True when no agent is predicted within [-gap_back, +gap_ahead] of the
  // ego along the route in the lane band around d_lane over gap_horizon.
  bool GapClear(const SceneInput& in, double s_ego, double d_lane,
                double half_width, double back_m, double ahead_m) const;
  // Stop line candidates on the route ahead: the nearest red / yellow-stop
  // / unknown-stop light, unhonoured stop sign, and the goal.
  std::optional<double> StopLineAhead(const SceneInput& in, double s_ego,
                                      double v, std::string* reason);
  // Dilemma-zone rule for a light seen yellow (M1 §3.2), latched per light.
  bool YellowMeansStop(const TrafficLightObs& light, double d_stop, double v);
  // The nearest predicted crossing of the ego corridor before the ego gets
  // there; nullopt when none.
  std::optional<double> ConflictAhead(const SceneInput& in, double s_ego,
                                      double v, double half_width,
                                      std::string* reason) const;
  // IDM desired speed behind `lead` (target = v + a_idm horizon).
  double IdmTargetSpeed(double v, double gap_m, double v_lead,
                        double v_limit) const;

  BehaviorFsmOptions options_;
  // Cross-tick state.
  std::optional<double> last_s_;
  Lateral committed_ = Lateral::kKeep;
  std::uint32_t committed_lane_ = 0;
  double commit_age_s_ = 0.0;
  double slow_lead_age_s_ = 0.0;
  std::map<std::uint32_t, bool> yellow_latch_;  // light id -> stop?
  std::map<std::uint32_t, TrafficLightColor> last_color_;
  std::set<std::uint32_t> honoured_signs_;
  std::uint32_t dwelling_sign_ = 0;
  double dwell_age_s_ = 0.0;
  // The yield latch: once a conflict is seen the decision holds its stop_s
  // for yield_hold_s, so a crossing agent flickering at the edge of the
  // prediction horizon does not flip free / yield every tick (task 17).
  double yield_age_s_ = 0.0;
  std::optional<double> yield_stop_s_;
  std::string yield_reason_;
};

}  // namespace nuway_planning

#endif  // NUWAY_PLANNING_BEHAVIOR_FSM_H_
