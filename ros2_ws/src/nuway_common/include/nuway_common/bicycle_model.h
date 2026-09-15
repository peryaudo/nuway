// Kinematic bicycle with a first-order steering lag (M1 §3.8): the model the
// MPC linearizes and the delay compensation integrates. State
// x = [X, Y, psi, v, delta] (map position of the rear axle in m, heading in
// rad, body speed in m/s, front wheel angle in rad), input u = [a, delta_cmd]
// (m/s^2, rad):
//
//   X' = v cos psi,  Y' = v sin psi,  psi' = v tan(delta) / L,
//   v' = a - a_off,  delta' = (delta_cmd - delta) / tau
//
// a_off is a constant acceleration disturbance (a road grade, the pedal
// map's residual) the controller estimates online and folds into the
// model, which is what gives the MPC offset-free tracking (mpc.h). The
// rear axle moves along its heading (no slip), the yaw rate follows from
// the turning circle of curvature tan(delta) / L (see pure_pursuit_pid.h for
// the geometry), and the wheel angle lags the command with the time constant
// tau measured by the sysid step responses (M0 §2.6). Discretised with the
// explicit midpoint rule (RK2): one Euler half-step to the midpoint, then a
// full step with the midpoint slope, second-order accurate and cheap enough
// to evaluate a few hundred times per tick. The steering lag is the stiff
// part: with the Lincoln's tau = 0.05 s and dt = 0.05 s the midpoint rule
// is stable (its amplification 1 - r + r^2/2 stays below 1 for r = dt/tau
// < 2) but decays the wheel-angle error by 50 % per step where the exact
// lag decays it to exp(-1) = 37 %, i.e. the model expects the wheel to lag
// the command slightly longer than it does. The Jacobians of the discrete
// step are exact for that scheme (chain rule through the midpoint), which
// is what makes the linearised prediction in the MPC consistent with the
// nonlinear rollout used for the delay compensation. No rclcpp; the M6
// expert reuses it through nuway_py. Introduced in M1.
#ifndef NUWAY_COMMON_BICYCLE_MODEL_H_
#define NUWAY_COMMON_BICYCLE_MODEL_H_

#include <cmath>

#include <Eigen/Core>

namespace nuway_common {

constexpr int kBicycleStateDim = 5;
constexpr int kBicycleInputDim = 2;

using BicycleState = Eigen::Matrix<double, kBicycleStateDim, 1>;
using BicycleInput = Eigen::Matrix<double, kBicycleInputDim, 1>;
using BicycleStateJacobian =
    Eigen::Matrix<double, kBicycleStateDim, kBicycleStateDim>;
using BicycleInputJacobian =
    Eigen::Matrix<double, kBicycleStateDim, kBicycleInputDim>;

// State and input indices, so the callers never count columns.
constexpr int kBicycleX = 0;
constexpr int kBicycleY = 1;
constexpr int kBicyclePsi = 2;
constexpr int kBicycleV = 3;
constexpr int kBicycleDelta = 4;
constexpr int kBicycleAccel = 0;
constexpr int kBicycleDeltaCmd = 1;

// The two physical parameters of the model.
struct BicycleParams {
  double wheelbase_m = 2.86;
  double tau_steer_s = 0.1;
  double accel_offset_mps2 = 0.0;  // a_off: v' = a - a_off
};

// Continuous-time derivative f(x, u).
inline BicycleState BicycleDerivative(const BicycleState& x,
                                      const BicycleInput& u,
                                      const BicycleParams& p) {
  BicycleState dx;
  const double v = x[kBicycleV];
  dx[kBicycleX] = v * std::cos(x[kBicyclePsi]);
  dx[kBicycleY] = v * std::sin(x[kBicyclePsi]);
  dx[kBicyclePsi] = v * std::tan(x[kBicycleDelta]) / p.wheelbase_m;
  dx[kBicycleV] = u[kBicycleAccel] - p.accel_offset_mps2;
  dx[kBicycleDelta] = (u[kBicycleDeltaCmd] - x[kBicycleDelta]) / p.tau_steer_s;
  return dx;
}

// df/dx at (x, u): the only nonlinear terms are the heading trigonometry
// and tan(delta), so the matrix has six non-zero entries.
inline BicycleStateJacobian BicycleDfDx(const BicycleState& x,
                                        const BicycleParams& p) {
  BicycleStateJacobian j = BicycleStateJacobian::Zero();
  const double v = x[kBicycleV];
  const double sin_psi = std::sin(x[kBicyclePsi]);
  const double cos_psi = std::cos(x[kBicyclePsi]);
  const double cos_delta = std::cos(x[kBicycleDelta]);
  j(kBicycleX, kBicyclePsi) = -v * sin_psi;
  j(kBicycleX, kBicycleV) = cos_psi;
  j(kBicycleY, kBicyclePsi) = v * cos_psi;
  j(kBicycleY, kBicycleV) = sin_psi;
  j(kBicyclePsi, kBicycleV) = std::tan(x[kBicycleDelta]) / p.wheelbase_m;
  j(kBicyclePsi, kBicycleDelta) = v / (p.wheelbase_m * cos_delta * cos_delta);
  j(kBicycleDelta, kBicycleDelta) = -1.0 / p.tau_steer_s;
  return j;
}

// df/du: constant, the model is affine in the input.
inline BicycleInputJacobian BicycleDfDu(const BicycleParams& p) {
  BicycleInputJacobian j = BicycleInputJacobian::Zero();
  j(kBicycleV, kBicycleAccel) = 1.0;
  j(kBicycleDelta, kBicycleDeltaCmd) = 1.0 / p.tau_steer_s;
  return j;
}

// One explicit-midpoint (RK2) step of length dt.
inline BicycleState BicycleStepRk2(const BicycleState& x, const BicycleInput& u,
                                   double dt_s, const BicycleParams& p) {
  const BicycleState k1 = BicycleDerivative(x, u, p);
  const BicycleState mid = x + ((0.5 * dt_s) * k1);
  const BicycleState k2 = BicycleDerivative(mid, u, p);
  return x + (dt_s * k2);
}

// The RK2 step and its exact Jacobians A = dF/dx, B = dF/du at (x, u):
// with m = x + dt/2 f(x, u) the step is F = x + dt f(m, u), so
//   A = I + dt J(m) (I + dt/2 J(x)),   B = dt (I + dt/2 J(m)) Bc,
// where J is df/dx and Bc the constant df/du.
struct BicycleLinearization {
  BicycleState x_next;
  BicycleStateJacobian a;
  BicycleInputJacobian b;
};

inline BicycleLinearization BicycleLinearizeRk2(const BicycleState& x,
                                                const BicycleInput& u,
                                                double dt_s,
                                                const BicycleParams& p) {
  const BicycleState k1 = BicycleDerivative(x, u, p);
  const BicycleState mid = x + ((0.5 * dt_s) * k1);
  const BicycleStateJacobian j_x = BicycleDfDx(x, p);
  const BicycleStateJacobian j_mid = BicycleDfDx(mid, p);
  const BicycleInputJacobian b_c = BicycleDfDu(p);
  const BicycleStateJacobian identity = BicycleStateJacobian::Identity();
  BicycleLinearization out;
  out.x_next = x + (dt_s * BicycleDerivative(mid, u, p));
  out.a = identity + (dt_s * (j_mid * (identity + ((0.5 * dt_s) * j_x))));
  out.b = dt_s * ((identity + ((0.5 * dt_s) * j_mid)) * b_c);
  return out;
}

}  // namespace nuway_common

#endif  // NUWAY_COMMON_BICYCLE_MODEL_H_
