#include "nuway_planning/safety_layer.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <nuway_common/geometry.h>
#include <nuway_common/tick.h>

namespace nuway_planning {
namespace {

using nuway_common::Trajectory;
using nuway_common::TrajectoryPoint;

constexpr double kEps = 1e-9;

}  // namespace

SafetyOptions SafetyOptions::FromVehicleModel(
    const nuway_control::VehicleModel& m) {
  SafetyOptions o;
  o.collision = CollisionOptions::FromVehicleModel(m);
  o.collision.margin_lon_m *= 2.0;
  o.collision.margin_lat_m *= 2.0;
  o.collision.agent_margin_m *= 2.0;
  o.a_min_mps2 = m.limits.a_min_mps2;
  o.a_max_mps2 = m.limits.a_max_mps2;
  o.kappa_phys = m.PhysicalCurvatureMax();
  return o;
}

Trajectory Retime(const Trajectory& traj, double age_s) {
  Trajectory out;
  out.reserve(nuway_common::kTrajectoryPoints);
  if (traj.empty()) {
    return out;
  }
  const TrajectoryPoint& last = traj.back();
  for (int i = 0; i < nuway_common::kTrajectoryPoints; ++i) {
    const double t = i * nuway_common::kTrajectoryDtS;
    const double t_in = t + age_s;
    TrajectoryPoint p;
    if (t_in <= last.t) {
      p = nuway_common::Interpolate(traj, t_in);
    } else {
      // Past the end: the terminal state continued along the heading.
      const double dt = t_in - last.t;
      p = last;
      p.x = last.x + (last.v * std::cos(last.yaw) * dt);
      p.y = last.y + (last.v * std::sin(last.yaw) * dt);
      p.a = 0.0;
    }
    p.t = t;
    out.push_back(p);
  }
  return out;
}

Trajectory StopAlongPath(const Trajectory& path, double v0_mps,
                         double decel_mps2) {
  Trajectory out;
  out.reserve(nuway_common::kTrajectoryPoints);
  if (path.empty()) {
    return out;
  }
  // Arc length of the path's points.
  std::vector<double> arc(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    arc[i] = arc[i - 1] +
             std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  const auto at_arc = [&](double s) {
    if (s <= 0.0 || path.size() == 1) {
      return path.front();
    }
    if (s >= arc.back()) {
      // Beyond the path: continue along the last heading.
      TrajectoryPoint p = path.back();
      const double extra = s - arc.back();
      p.x += extra * std::cos(p.yaw);
      p.y += extra * std::sin(p.yaw);
      return p;
    }
    const auto upper = std::upper_bound(arc.begin(), arc.end(), s);
    const auto j = static_cast<std::size_t>(upper - arc.begin());
    const std::size_t i = j - 1;
    const double span = arc[j] - arc[i];
    const double a = span > kEps ? (s - arc[i]) / span : 0.0;
    TrajectoryPoint p;
    p.x = path[i].x + (a * (path[j].x - path[i].x));
    p.y = path[i].y + (a * (path[j].y - path[i].y));
    p.yaw = nuway_common::WrapAngle(
        path[i].yaw + (a * nuway_common::WrapAngle(path[j].yaw - path[i].yaw)));
    p.kappa = path[i].kappa + (a * (path[j].kappa - path[i].kappa));
    return p;
  };
  const double v0 = std::max(0.0, v0_mps);
  const double decel = std::max(decel_mps2, kEps);
  const double t_stop = v0 / decel;
  for (int i = 0; i < nuway_common::kTrajectoryPoints; ++i) {
    const double t = i * nuway_common::kTrajectoryDtS;
    double s = 0.0;
    double v = 0.0;
    double a = 0.0;
    if (t < t_stop) {
      s = (v0 * t) - (0.5 * decel * t * t);
      v = v0 - (decel * t);
      a = -decel;
    } else {
      s = 0.5 * v0 * t_stop;
    }
    TrajectoryPoint p = at_arc(s);
    p.t = t;
    p.v = v;
    p.a = a;
    out.push_back(p);
  }
  return out;
}

SafetyLayer::SafetyLayer(SafetyOptions options)
    : options_(options), checker_(options.collision) {}

void SafetyLayer::Reset() {
  last_safe_.reset();
  held_.reset();
}

bool SafetyLayer::Collides(const Trajectory& traj,
                           const SafetyInput& in) const {
  static const std::vector<nuway_common::AgentState> kNoAgents;
  static const nuway_common::PredictionSet kNoPredictions;
  const CollisionResult r = checker_.Check(
      traj, in.agents != nullptr ? *in.agents : kNoAgents,
      in.predictions != nullptr ? *in.predictions : kNoPredictions);
  return r.collides() && r.min_ttc_s <= options_.collision_horizon_s;
}

bool SafetyLayer::OccupancyHit(const Trajectory& traj,
                               const OccupancyView& grid) const {
  const std::size_t cells = static_cast<std::size_t>(grid.spec.height) *
                            static_cast<std::size_t>(grid.spec.width);
  if (grid.occupied.size() != cells || cells == 0) {
    return false;
  }
  const nuway_common::SE2 map_to_base = nuway_common::Inverse(grid.base_pose);
  const double step = std::max(options_.footprint_sample_m, 0.1);
  for (const TrajectoryPoint& p : traj) {
    if (p.t > options_.occupancy_horizon_s) {
      break;
    }
    const OrientedBox box = checker_.EgoBoxAt(traj, p.t, false);
    const double c = std::cos(box.yaw_rad);
    const double s = std::sin(box.yaw_rad);
    // Sample points across the footprint on a `step` lattice.
    const int nl =
        std::max(1, static_cast<int>(std::ceil(box.length_m / step)));
    const int nw = std::max(1, static_cast<int>(std::ceil(box.width_m / step)));
    for (int i = 0; i <= nl; ++i) {
      const double u = (-0.5 * box.length_m) + (box.length_m * i / nl);
      for (int j = 0; j <= nw; ++j) {
        const double w = (-0.5 * box.width_m) + (box.width_m * j / nw);
        const Eigen::Vector2d map_pt =
            box.center + Eigen::Vector2d((u * c) - (w * s), (u * s) + (w * c));
        const Eigen::Vector2d base_pt =
            nuway_common::Apply(map_to_base, map_pt);
        const double occ = nuway_common::BilinearSample(
            grid.spec, grid.occupied.data(), base_pt.x(), base_pt.y(), 0.0);
        if (occ >= options_.occupancy_threshold) {
          return true;
        }
      }
    }
  }
  return false;
}

bool SafetyLayer::EnforceLimits(Trajectory* traj) const {
  bool clipped = false;
  for (TrajectoryPoint& p : *traj) {
    const double a = std::clamp(p.a, options_.a_min_mps2, options_.a_max_mps2);
    const double kappa =
        std::clamp(p.kappa, -options_.kappa_phys, options_.kappa_phys);
    if (std::abs(a - p.a) > kEps || std::abs(kappa - p.kappa) > kEps) {
      clipped = true;
    }
    p.a = a;
    p.kappa = kappa;
  }
  if (!clipped || traj->size() < 2) {
    return clipped;
  }
  // Re-integrate the kinematic bicycle from the first point with the
  // clipped inputs so the samples stay mutually consistent.
  for (std::size_t i = 0; i + 1 < traj->size(); ++i) {
    const TrajectoryPoint& p = (*traj)[i];
    TrajectoryPoint& q = (*traj)[i + 1];
    const double dt = q.t - p.t;
    q.v = std::max(0.0, p.v + (p.a * dt));
    const double v_mid = 0.5 * (p.v + q.v);
    q.yaw = nuway_common::WrapAngle(p.yaw + (v_mid * p.kappa * dt));
    const double yaw_mid = p.yaw + (0.5 * v_mid * p.kappa * dt);
    q.x = p.x + (v_mid * std::cos(yaw_mid) * dt);
    q.y = p.y + (v_mid * std::sin(yaw_mid) * dt);
  }
  return true;
}

SafetyOutput SafetyLayer::Step(const SafetyInput& in) {
  SafetyOutput out;
  Trajectory traj;
  if (in.planner_degraded) {
    // Step 1: the gentlest stop along the last safe path that clears the
    // check, held for the rest of the episode.
    out.intervened = true;
    out.reason = "degraded";
    out.source = "fallback";
    if (held_.has_value()) {
      held_ = Retime(*held_, nuway_common::kTickDtS);
    } else if (last_safe_.has_value()) {
      Trajectory gentle =
          StopAlongPath(*last_safe_, in.ego_speed_mps, options_.a_gentle_mps2);
      if (Collides(gentle, in)) {
        gentle =
            StopAlongPath(*last_safe_, in.ego_speed_mps, -options_.a_min_mps2);
        out.reason = "degraded:hard";
      }
      held_ = std::move(gentle);
    } else {
      // The planner died before its first output: the car never moved.
      held_ = nuway_common::StopTrajectory(in.ego_pose);
      out.source = "none";
    }
    out.trajectory = *held_;
    return out;
  }
  if (in.trajectory == nullptr || in.trajectory->empty() ||
      in.source == "none") {
    // The no-input trajectory passes through as a stop at the pose.
    out.trajectory = nuway_common::StopTrajectory(in.ego_pose);
    out.source = "none";
    return out;
  }
  traj = Retime(*in.trajectory, in.trajectory_age_s);
  out.source = in.source;
  // Step 2: collision with doubled margins over the first seconds.
  if (Collides(traj, in)) {
    traj = StopAlongPath(traj, in.ego_speed_mps, -options_.a_min_mps2);
    out.intervened = true;
    out.reason = "collision";
    out.source = "fallback";
  }
  // Step 3: limits.
  if (EnforceLimits(&traj)) {
    out.intervened = true;
    out.reason = out.reason.empty() ? "limits" : out.reason + ",limits";
  }
  // Step 4: occupancy footprint.
  if (in.occupancy != nullptr && OccupancyHit(traj, *in.occupancy)) {
    traj = StopAlongPath(traj, in.ego_speed_mps, -options_.a_min_mps2);
    out.intervened = true;
    out.reason = out.reason.empty() ? "occupancy" : out.reason + ",occupancy";
    out.source = "fallback";
  }
  last_safe_ = traj;
  out.trajectory = std::move(traj);
  return out;
}

}  // namespace nuway_planning
