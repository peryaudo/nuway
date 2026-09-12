// Linear time-varying MPC on the kinematic bicycle (M1 §3.8): the M1+
// controller, a pure library the node feeds one tick at a time. No rclcpp;
// nuway_py binds it in M6.
//
// Model: the bicycle with steering lag of nuway_common/bicycle_model.h,
// state x = [X, Y, psi, v, delta], input u = [a, delta_cmd], stepped with
// RK2 at dt = 0.05 s over N = 20 steps (1 s). The reference is the safety
// layer's trajectory resampled at dt from t_delay on, with delta_ref =
// atan(L kappa) and a_ref = a of each sample; the model is linearised about
// every reference knot (A_k, B_k from the exact RK2 Jacobians, the defect
// d_k = F(x_ref_k, u_ref_k) - x_ref_{k+1} kept, so a reference the model
// cannot follow exactly is still predicted correctly to first order).
//
// Offset-free tracking: the kinematic model has no road grade and the pedal
// map has a 0.4 m/s^2 residual (M0 §2.6), so the node estimates an
// acceleration disturbance a_off = a_applied - a_measured (first-order
// low-pass over accel_bias_tau_s, from the command that was acting during
// the last tick and EgoState.ax) and the model integrates v' = a - a_off.
// The QP then commands a_ref + a_off on its own — integral action in the
// model rather than a bolt-on integrator — which is what keeps the car
// climbing the 15 % ramp at the start of the Town03 dev routes. The
// estimate ignores a measurement beyond twice the vehicle's limits (the
// tick the wheels lock reports EgoState.ax of tens of m/s^2) and, while
// the brake acts instead of the pedal map (after an emergency stop, or
// braking below accel_bias_stop_speed_mps), relaxes to zero: the brake-
// side residual would otherwise hold the car at a stop line as a throttle
// surplus. It is bounded by accel_bias_max_mps2.
//
// Delay compensation: the measured state is rolled forward by t_delay
// (default 0.10 s = 2 ticks, perception to actuation) with the commands
// already issued but not yet acting, and the QP starts from the result
// against the reference shifted by the same t_delay. Without it the
// controller would react to a state two ticks old and oscillate at speed.
//
// QP: with e_k = x_k - x_ref_k expressed in the reference knot's local
// frame (longitudinal, lateral, heading, speed, wheel angle) so that Q can
// weight lateral error eight times the longitudinal one,
//   min sum_k e_k^T Q e_k + (u_k - u_ref_k)^T R (u_k - u_ref_k)
//              + (u_k - u_{k-1})^T R_d (u_k - u_{k-1})
//   s.t. a in [a_min, a_max], delta_cmd in [-delta_max, delta_max],
//        -jerk_brake_max dt <= a_k - a_{k-1} <= jerk_max dt,
//        |delta_k - delta_{k-1}| <= rate dt,
// with u_{-1} the last published command and the last knot weighted by
// q_terminal_scale. The states are eliminated (condensed) through the
// linear dynamics, leaving a dense QP over the 2N = 40 inputs with 4N box
// and rate rows, solved by OSQP through osqp-eigen with fixed settings
// (adaptive rho off, fixed max_iter, polish on, §5), set up afresh every
// tick so its equilibration matches the tick's problem, and warm-started
// from the previous solution shifted by one step. Only inputs are constrained,
// so the QP is always feasible and bounded; "solver failure" means the
// iteration budget ran out.
//
// Solver outcomes (§3.8): a converged solve, including one whose polish
// step failed, is used as is; an exhausted budget uses the returned
// iterate clamped to the input and rate bounds and counts one failure;
// no usable iterate (solver error, non-finite values) is an emergency stop
// at once. After max_consecutive_solver_failures counted failures in a
// row the controller publishes emergency_stop instead of the iterate until
// a solve converges again (node-local, counted in ticks, deterministic).
// Every emergency stop carries the delta_ref of the knot being tracked as
// its steering angle so the brake does not straighten the wheel in a
// curve. The failure counter, the warm start and the command history are
// dropped by Reset() (ResetEvent).
#ifndef NUWAY_CONTROL_MPC_H_
#define NUWAY_CONTROL_MPC_H_

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include <nuway_common/geometry.h>
#include <nuway_common/trajectory.h>

#include "nuway_control/vehicle_model.h"

namespace nuway_control {

// What one tick's solve came to; the §3.8 policy keys on it.
enum class MpcOutcome : std::uint8_t {
  kSolved,     // converged (a failed polish step still counts)
  kBudget,     // iteration budget out: the clamped iterate was used
  kNoIterate,  // no usable iterate: emergency stop at once
  kNoInput,    // no QP ran (invalid pose, no or degraded reference)
};

const char* MpcOutcomeName(MpcOutcome outcome);

// OSQP's termination status reduced to what the policy distinguishes.
enum class MpcSolveStatus : std::uint8_t {
  kSolved,
  kSolvedInaccurate,
  kMaxIterReached,
  kOther,  // infeasible / unbounded / non-convex / unsolved: not expected
};

// The solver-outcome policy on one solve: `polish_status` is OSQP's (1
// success, 0 not run, -1 failed) and is deliberately ignored, `finite`
// whether the returned iterate has finite entries.
MpcOutcome ClassifySolve(MpcSolveStatus status, int polish_status, bool finite);

struct MpcQpSettings {
  double eps_abs = 1e-4;
  double eps_rel = 1e-4;
  // The fixed budget of §5; chosen and measured in the task 17 tuning
  // note (M1 Decisions log): a tight budget turns slow ticks into stops.
  int max_iter = 400;
  int check_termination = 5;
  double rho = 0.1;  // fixed ADMM step (adaptive rho off, §5)
  double sigma = 1e-6;
  int scaling = 10;
  bool polish = true;
  bool warm_start = true;
};

// The knobs of §3.8; config/defaults.yaml mirrors them.
struct MpcOptions {
  int horizon = 20;         // N steps of dt_s
  double dt_s = 0.05;       // the tick
  double t_delay_s = 0.10;  // perception-to-actuation delay compensated
  // Q = diag(lon, lat, yaw, v, delta) in the reference knot's local frame.
  std::array<double, 5> q = {0.5, 4.0, 2.0, 0.5, 0.0};
  double q_terminal_scale = 1.0;            // the last knot's Q multiplier
  std::array<double, 2> r = {0.1, 1.0};     // on u - u_ref
  std::array<double, 2> r_d = {1.0, 10.0};  // on u_k - u_{k-1}
  int max_consecutive_solver_failures = 5;
  // Acceleration disturbance observer (0 disables it).
  double accel_bias_tau_s = 0.5;
  double accel_bias_max_mps2 = 3.0;
  double accel_bias_stop_speed_mps = 1.0;  // no update while braking below
  // The measured wheel angle (EgoState.steering_angle, the CARLA front
  // wheels in our harness) when true; the steering-lag model driven by
  // this controller's own commands when false (§3.8, Leaderboard).
  bool use_measured_steering = true;
  MpcQpSettings qp;
};

struct MpcInput {
  nuway_common::SE2 pose;   // base_link (rear axle) in map
  double speed_mps = 0.0;   // body-frame longitudinal speed
  double accel_mps2 = 0.0;  // measured body-frame longitudinal accel
  double steering_angle_rad = 0.0;
  bool pose_valid = true;
  // The safe trajectory of this tick (t relative to now); nullptr or
  // degraded is the no-input case.
  const nuway_common::Trajectory* trajectory = nullptr;
  bool trajectory_degraded = false;
};

struct MpcOutput {
  double accel_mps2 = 0.0;
  double steering_angle_rad = 0.0;  // delta_cmd, or delta_ref on a stop
  bool emergency_stop = true;
  // ControlDebug fields, measured against the reference at t = 0.
  double lateral_error_m = 0.0;    // left positive
  double heading_error_rad = 0.0;  // reference heading - ego yaw
  double speed_error_mps = 0.0;    // reference - measured
  double solve_time_ms = 0.0;      // the OSQP solve alone
  bool solver_ok = false;          // false on a counted failure or a stop
  MpcOutcome outcome = MpcOutcome::kNoInput;
  int iterations = 0;
  bool counted_failure = false;
  int consecutive_failures = 0;
  double delta_ref_rad = 0.0;    // of the knot tracked (or the last valid)
  double accel_bias_mps2 = 0.0;  // the disturbance estimate in use
  std::string status;            // OSQP's status name or the no-input reason
  // The predicted horizon: N + 1 points at dt from t_delay, the first
  // being the delay-compensated state (marker_node's mpc_horizon layer).
  nuway_common::Trajectory horizon;
};

class Mpc {
 public:
  Mpc(VehicleModel model, MpcOptions options);
  ~Mpc();
  Mpc(const Mpc&) = delete;
  Mpc& operator=(const Mpc&) = delete;
  Mpc(Mpc&&) noexcept;
  Mpc& operator=(Mpc&&) noexcept;

  // One tick.
  MpcOutput Step(const MpcInput& in);
  // Drops the warm start, the command history, the lag-model wheel angle,
  // the disturbance estimate and the failure counter (ResetEvent).
  void Reset();
  // Re-tunes the iteration budget of the live solver (tests).
  void set_max_iter(int max_iter);

  const MpcOptions& options() const;
  const VehicleModel& model() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nuway_control

#endif  // NUWAY_CONTROL_MPC_H_
