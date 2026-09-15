// Constant-velocity agent prediction with an optional lane-follow mode
// (M1 §3.1). Pure library (no rclcpp): const_vel_node feeds it the map-frame
// agents of a tick and publishes the PredictionSet it returns; nuway_py
// binds it for the M6 expert.
//
// The model. Prediction here is the simplest physically meaningful prior:
// every agent keeps doing what it does now. For S = 1 sample and T steps of
// dt seconds, position(t) = p + v t and yaw stays constant, which is exact
// for straight roads and wrong at every bend, where it sends the predicted
// car off the road and into the ego's lane. `lane_follow` fixes the bend
// case for vehicles without a learned model: the agent is snapped to the
// nearest drivable lane (LaneGraph::NearestLane, a KD-tree over centerline
// samples with a heading check so an oncoming car is matched to its own
// lane), its velocity is projected onto the lane direction, and the future
// is walked along the lane centerline at that speed, keeping the agent's
// current lateral offset, and on through the successor lane whose entry
// heading continues the current one most smoothly. Where the lane graph
// ends (no successor) the walk continues straight. Pedestrians are never
// lane-following and their speed is clamped to pedestrian_speed_max (a
// walker's velocity spikes when it turns); static obstacles stay put. The
// price of the lane-follow prior is a wrong guess whenever a car is about
// to turn off its lane; M7's flow-matching model is what replaces this.
// Time index t (0-based) is (t + 1) dt after the input stamp: the current
// pose is the input itself, so the first predicted step is dt ahead.
#ifndef NUWAY_PREDICTION_CONST_VEL_H_
#define NUWAY_PREDICTION_CONST_VEL_H_

#include <cstdint>
#include <optional>
#include <vector>

#include <nuway_common/agents.h>
#include <nuway_common/geometry.h>
#include <nuway_map/lane_graph.h>

namespace nuway_prediction {

// Tuning knobs; mirrored by config/defaults.yaml (the node declares each as
// a parameter).
struct ConstVelOptions {
  int num_timesteps = 16;  // T: 8 s at 0.5 s
  double dt_s = 0.5;
  bool lane_follow = true;                 // vehicles follow their lane
  double pedestrian_speed_max_mps = 2.0;   // walker speed clamp
  double lane_search_dist_m = 3.0;         // NearestLane radius
  double lane_follow_min_speed_mps = 0.3;  // slower along the lane: straight
};

// One agent's predicted poses, map frame, poses[t] at (t + 1) dt.
struct AgentFuture {
  std::uint32_t id = 0;
  std::vector<nuway_common::SE2> poses;
};

class ConstVelPredictor {
 public:
  explicit ConstVelPredictor(ConstVelOptions options);

  // The lane graph for lane_follow; non-owning, may be null (straight-line
  // prediction for everyone until one is set).
  void set_lane_graph(const nuway_map::LaneGraph* graph) { graph_ = graph; }
  const ConstVelOptions& options() const { return options_; }

  // Predicts every agent (map frame) as one joint sample of weight 1.
  nuway_common::PredictionSet Predict(
      const std::vector<nuway_common::AgentState>& agents) const;

  // Predicts one agent: lane-follow for a moving vehicle on a lane, the
  // straight line otherwise.
  AgentFuture PredictAgent(const nuway_common::AgentState& agent) const;

 private:
  // p + v t with the speed capped at `speed_cap_mps` (a negative cap means
  // no cap) and the yaw held.
  AgentFuture Straight(const nuway_common::AgentState& agent,
                       double speed_cap_mps) const;
  // The lane-follow walk; nullopt when the agent is on no lane, moves
  // against it, or is too slow along it to matter.
  std::optional<AgentFuture> AlongLane(
      const nuway_common::AgentState& agent) const;
  // The successor whose entry heading continues `heading_rad` most
  // smoothly; 0 when the lane has none.
  std::uint32_t SmoothestSuccessor(std::uint32_t lane_id,
                                   double heading_rad) const;

  ConstVelOptions options_;
  const nuway_map::LaneGraph* graph_ = nullptr;
};

}  // namespace nuway_prediction

#endif  // NUWAY_PREDICTION_CONST_VEL_H_
