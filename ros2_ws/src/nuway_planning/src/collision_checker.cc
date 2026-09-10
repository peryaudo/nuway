#include "nuway_planning/collision_checker.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <nuway_common/geometry.h>

namespace nuway_planning {
namespace {

using nuway_common::AgentState;
using nuway_common::PredictionSet;
using nuway_common::SE2;
using nuway_common::Trajectory;

// Half the diagonal: the radius of the circle bounding the box, for the
// cheap rejection before the SAT.
double BoundingRadius(const OrientedBox& box) {
  return 0.5 * std::hypot(box.length_m, box.width_m);
}

// Projection interval of the box's corners onto a unit axis.
void Project(const std::array<Eigen::Vector2d, 4>& corners,
             const Eigen::Vector2d& axis, double* lo, double* hi) {
  *lo = std::numeric_limits<double>::infinity();
  *hi = -std::numeric_limits<double>::infinity();
  for (const Eigen::Vector2d& c : corners) {
    const double p = c.dot(axis);
    *lo = std::min(*lo, p);
    *hi = std::max(*hi, p);
  }
}

// The agent's footprint at a check time: the observed pose at t = 0, the
// prediction step nearest to t afterwards (held at the last step past the
// prediction horizon), or the observed pose throughout when the set does
// not carry the agent.
OrientedBox AgentBoxAt(const AgentState& agent, const PredictionSet& set,
                       int agent_index, int sample, double t, double margin) {
  OrientedBox box;
  box.length_m = agent.length_m + (2.0 * margin);
  box.width_m = agent.width_m + (2.0 * margin);
  SE2 pose = agent.pose;
  if (t > 0.0 && agent_index >= 0 && set.num_timesteps > 0 && set.dt_s > 0.0) {
    const int step = std::clamp(static_cast<int>(std::lround(t / set.dt_s)) - 1,
                                0, set.num_timesteps - 1);
    pose = set.PoseAt(sample, agent_index, step);
  }
  box.center = Eigen::Vector2d(pose.x, pose.y);
  box.yaw_rad = pose.yaw;
  return box;
}

}  // namespace

std::array<Eigen::Vector2d, 4> OrientedBox::Corners() const {
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  const Eigen::Vector2d fwd(c, s);
  const Eigen::Vector2d left(-s, c);
  const Eigen::Vector2d half_l = 0.5 * length_m * fwd;
  const Eigen::Vector2d half_w = 0.5 * width_m * left;
  return {center + half_l + half_w, center - half_l + half_w,
          center - half_l - half_w, center + half_l - half_w};
}

bool BoxesOverlap(const OrientedBox& a, const OrientedBox& b) {
  if ((a.center - b.center).norm() > BoundingRadius(a) + BoundingRadius(b)) {
    return false;
  }
  const std::array<Eigen::Vector2d, 4> ca = a.Corners();
  const std::array<Eigen::Vector2d, 4> cb = b.Corners();
  // For rectangles the candidate separating axes are the two edge
  // directions of each box (the edge normals are the same set).
  const std::array<Eigen::Vector2d, 4> axes = {
      Eigen::Vector2d(std::cos(a.yaw_rad), std::sin(a.yaw_rad)),
      Eigen::Vector2d(-std::sin(a.yaw_rad), std::cos(a.yaw_rad)),
      Eigen::Vector2d(std::cos(b.yaw_rad), std::sin(b.yaw_rad)),
      Eigen::Vector2d(-std::sin(b.yaw_rad), std::cos(b.yaw_rad))};
  for (const Eigen::Vector2d& axis : axes) {
    double lo_a = 0.0;
    double hi_a = 0.0;
    double lo_b = 0.0;
    double hi_b = 0.0;
    Project(ca, axis, &lo_a, &hi_a);
    Project(cb, axis, &lo_b, &hi_b);
    if (hi_a < lo_b || hi_b < lo_a) {
      return false;
    }
  }
  return true;
}

double DiscDistance(const OrientedBox& a, const OrientedBox& b) {
  const auto discs = [](const OrientedBox& box, double* radius) {
    const Eigen::Vector2d fwd(std::cos(box.yaw_rad), std::sin(box.yaw_rad));
    *radius = std::hypot(box.length_m / 6.0, box.width_m / 2.0);
    const double step = box.length_m / 3.0;
    return std::array<Eigen::Vector2d, 3>{box.center - (step * fwd), box.center,
                                          box.center + (step * fwd)};
  };
  double ra = 0.0;
  double rb = 0.0;
  const std::array<Eigen::Vector2d, 3> da = discs(a, &ra);
  const std::array<Eigen::Vector2d, 3> db = discs(b, &rb);
  double best = std::numeric_limits<double>::infinity();
  for (const Eigen::Vector2d& pa : da) {
    for (const Eigen::Vector2d& pb : db) {
      best = std::min(best, (pa - pb).norm() - ra - rb);
    }
  }
  return std::max(0.0, best);
}

CollisionOptions CollisionOptions::FromVehicleModel(
    const nuway_control::VehicleModel& m) {
  CollisionOptions o;
  o.ego_length_m = m.length_m;
  o.ego_width_m = m.width_m;
  o.ego_center_offset_m = -m.rear_axle_offset_x_m;
  return o;
}

CollisionChecker::CollisionChecker(CollisionOptions options)
    : options_(options) {}

OrientedBox CollisionChecker::EgoBoxAt(const Trajectory& trajectory, double t,
                                       bool inflated) const {
  const nuway_common::TrajectoryPoint p =
      nuway_common::Interpolate(trajectory, t);
  OrientedBox box;
  box.yaw_rad = p.yaw;
  box.center = Eigen::Vector2d(p.x, p.y) +
               (options_.ego_center_offset_m *
                Eigen::Vector2d(std::cos(p.yaw), std::sin(p.yaw)));
  box.length_m =
      options_.ego_length_m + (inflated ? 2.0 * options_.margin_lon_m : 0.0);
  box.width_m =
      options_.ego_width_m + (inflated ? 2.0 * options_.margin_lat_m : 0.0);
  return box;
}

CollisionResult CollisionChecker::Check(
    const Trajectory& trajectory, const std::vector<AgentState>& agents,
    const PredictionSet& predictions) const {
  CollisionResult result;
  result.min_ttc_s = std::numeric_limits<double>::infinity();
  const int steps = options_.check_dt_s > 0.0
                        ? static_cast<int>(std::lround(options_.horizon_s /
                                                       options_.check_dt_s))
                        : 0;
  for (int k = 0; k <= steps; ++k) {
    result.times_s.push_back(k * options_.check_dt_s);
  }
  result.min_distance_m.assign(result.times_s.size(),
                               std::numeric_limits<double>::infinity());
  if (trajectory.empty()) {
    return result;
  }
  const int samples = std::max(1, predictions.num_samples);
  std::vector<double> weights(static_cast<std::size_t>(samples), 1.0);
  if (predictions.sample_weight.size() == weights.size()) {
    weights = predictions.sample_weight;
  }
  double weight_sum = 0.0;
  for (const double w : weights) {
    weight_sum += w;
  }
  // Agent index in the set, -1 when absent (static at the observed pose).
  std::vector<int> index;
  index.reserve(agents.size());
  for (const AgentState& agent : agents) {
    index.push_back(predictions.IndexOf(agent.id));
  }
  double colliding_weight = 0.0;
  for (int s = 0; s < samples; ++s) {
    bool sample_collides = false;
    for (std::size_t k = 0; k < result.times_s.size(); ++k) {
      const double t = result.times_s[k];
      const OrientedBox ego = EgoBoxAt(trajectory, t, true);
      const OrientedBox ego_raw = EgoBoxAt(trajectory, t, false);
      for (std::size_t a = 0; a < agents.size(); ++a) {
        const bool in_set = index[a] >= 0 && s < predictions.num_samples;
        const OrientedBox box =
            AgentBoxAt(agents[a], predictions, in_set ? index[a] : -1, s, t,
                       options_.agent_margin_m);
        if (s == 0 || in_set) {
          // Static agents repeat across samples: measure them once.
          OrientedBox raw = box;
          raw.length_m = agents[a].length_m;
          raw.width_m = agents[a].width_m;
          result.min_distance_m[k] =
              std::min(result.min_distance_m[k], DiscDistance(ego_raw, raw));
        }
        if (BoxesOverlap(ego, box)) {
          sample_collides = true;
          result.min_ttc_s = std::min(result.min_ttc_s, t);
        }
      }
    }
    if (sample_collides) {
      colliding_weight += weights[static_cast<std::size_t>(s)];
    }
  }
  result.cost = weight_sum > 0.0 ? colliding_weight / weight_sum : 0.0;
  return result;
}

}  // namespace nuway_planning
