// Trajectory struct (the C++ image of nuway_msgs/Trajectory points), time
// interpolation and fixed-step resampling. Positions in the map frame, t in
// seconds relative to the trajectory stamp, v in m/s, a in m/s^2, kappa in 1/m.
// Introduced in M0; the spline resample arrives with M8.
#ifndef NUWAY_COMMON_TRAJECTORY_H_
#define NUWAY_COMMON_TRAJECTORY_H_

#include <algorithm>
#include <vector>

#include "nuway_common/geometry.h"

namespace nuway_common {

// One trajectory sample.
struct TrajectoryPoint {
  double t = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double v = 0.0;
  double a = 0.0;
  double kappa = 0.0;
};

using Trajectory = std::vector<TrajectoryPoint>;

// Number of points every producer emits: 0.1 s spacing over an 8 s horizon.
constexpr int kTrajectoryPoints = 81;
constexpr double kTrajectoryDtS = 0.1;

// Linear interpolation at time t (yaw interpolated on the circle). t outside
// the trajectory clamps to the first/last point. Empty input yields a default
// point.
inline TrajectoryPoint Interpolate(const Trajectory& traj, double t) {
  if (traj.empty()) {
    return TrajectoryPoint{};
  }
  if (t <= traj.front().t) {
    return traj.front();
  }
  if (t >= traj.back().t) {
    return traj.back();
  }
  const auto upper = std::upper_bound(
      traj.begin(), traj.end(), t,
      [](double value, const TrajectoryPoint& p) { return value < p.t; });
  const TrajectoryPoint& p1 = *upper;
  const TrajectoryPoint& p0 = *(upper - 1);
  const double span = p1.t - p0.t;
  const double alpha = span > 0.0 ? (t - p0.t) / span : 0.0;
  TrajectoryPoint out;
  out.t = t;
  out.x = p0.x + (alpha * (p1.x - p0.x));
  out.y = p0.y + (alpha * (p1.y - p0.y));
  out.yaw = WrapAngle(p0.yaw + (alpha * WrapAngle(p1.yaw - p0.yaw)));
  out.v = p0.v + (alpha * (p1.v - p0.v));
  out.a = p0.a + (alpha * (p1.a - p0.a));
  out.kappa = p0.kappa + (alpha * (p1.kappa - p0.kappa));
  return out;
}

// Resamples at t = t0 + i * dt for i in [0, n).
inline Trajectory Resample(const Trajectory& traj, double t0, double dt,
                           int n) {
  Trajectory out;
  out.reserve(static_cast<std::size_t>(std::max(0, n)));
  for (int i = 0; i < n; ++i) {
    out.push_back(Interpolate(traj, t0 + (i * dt)));
  }
  return out;
}

// The no-input trajectory of docs/02_interfaces.md §2: kTrajectoryPoints
// samples holding `pose` at v = 0.
inline Trajectory StopTrajectory(const SE2& pose) {
  Trajectory out;
  out.reserve(kTrajectoryPoints);
  for (int i = 0; i < kTrajectoryPoints; ++i) {
    TrajectoryPoint p;
    p.t = i * kTrajectoryDtS;
    p.x = pose.x;
    p.y = pose.y;
    p.yaw = pose.yaw;
    out.push_back(p);
  }
  return out;
}

}  // namespace nuway_common

#endif  // NUWAY_COMMON_TRAJECTORY_H_
