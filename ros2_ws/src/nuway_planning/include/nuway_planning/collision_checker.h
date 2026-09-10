// Collision checker (M1 §3.4): oriented-box overlap between the ego
// footprint swept along a candidate and every agent's predicted footprint,
// by the separating axis theorem, at t = 0, 0.5, ..., 8 s. t = 0 uses the
// agent's observed pose (map frame after the §3.1 frame rule), the later
// times the prediction sample's steps. The result carries the weighted
// fraction of joint samples that collide, the first colliding time
// (min_ttc) and a per-time minimum footprint distance from a 3-disc
// approximation for the selector's proximity term. No rclcpp.
#ifndef NUWAY_PLANNING_COLLISION_CHECKER_H_
#define NUWAY_PLANNING_COLLISION_CHECKER_H_

#include <array>
#include <vector>

#include <Eigen/Core>

#include <nuway_common/agents.h>
#include <nuway_common/trajectory.h>
#include <nuway_control/vehicle_model.h>

namespace nuway_planning {

// A rectangle of `length` along its heading and `width` across it, centred
// at `center`, map frame.
struct OrientedBox {
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  double yaw_rad = 0.0;
  double length_m = 0.0;
  double width_m = 0.0;

  // The four corners, counter-clockwise from the front-left.
  std::array<Eigen::Vector2d, 4> Corners() const;
};

// Separating axis theorem for two rectangles: they overlap unless one of
// the four edge normals separates their projections (a touching pair
// counts as overlapping).
bool BoxesOverlap(const OrientedBox& a, const OrientedBox& b);

// Minimum distance between two boxes approximated by three discs each,
// spaced along the length axis with radius sqrt((L/6)^2 + (W/2)^2) so the
// discs cover the rectangle; 0 when the discs overlap.
double DiscDistance(const OrientedBox& a, const OrientedBox& b);

struct CollisionOptions {
  double margin_lon_m = 1.0;    // ego inflation at the front and the rear
  double margin_lat_m = 0.4;    // ego inflation on each side
  double agent_margin_m = 0.2;  // agent inflation on every side
  double check_dt_s = 0.5;      // t = 0, dt, ..., horizon
  double horizon_s = 8.0;
  double ego_length_m = 4.892;
  double ego_width_m = 1.837;
  // Footprint centre ahead of base_link (the rear axle): -rear_axle_offset_x
  // of the vehicle YAML, since the CARLA box is centred on the actor
  // origin.
  double ego_center_offset_m = 1.389;

  static CollisionOptions FromVehicleModel(
      const nuway_control::VehicleModel& m);
};

struct CollisionResult {
  // Weighted fraction of joint samples with at least one collision, in
  // [0, 1] (uniform weights when the set carries none).
  double cost = 0.0;
  // First colliding time over every sample and agent, s; +infinity when
  // nothing collides.
  double min_ttc_s = 0.0;
  // Minimum footprint distance to any agent (uninflated, 3-disc
  // approximation) at each check time, m.
  std::vector<double> min_distance_m;
  // Check times, s.
  std::vector<double> times_s;

  bool collides() const { return cost > 0.0; }
};

class CollisionChecker {
 public:
  explicit CollisionChecker(CollisionOptions options);

  // Checks one candidate trajectory (rear-axle poses over t) against the
  // agents and their predictions. An agent absent from the prediction set
  // keeps its observed pose (static).
  CollisionResult Check(const nuway_common::Trajectory& trajectory,
                        const std::vector<nuway_common::AgentState>& agents,
                        const nuway_common::PredictionSet& predictions) const;

  // The (inflated when `inflated`) ego footprint at a trajectory time.
  OrientedBox EgoBoxAt(const nuway_common::Trajectory& trajectory, double t,
                       bool inflated) const;

  const CollisionOptions& options() const { return options_; }

 private:
  CollisionOptions options_;
};

}  // namespace nuway_planning

#endif  // NUWAY_PLANNING_COLLISION_CHECKER_H_
