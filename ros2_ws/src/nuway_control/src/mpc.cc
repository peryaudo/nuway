#include "nuway_control/mpc.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <OsqpEigen/OsqpEigen.h>

#include <nuway_common/bicycle_model.h>
#include <nuway_common/geometry.h>
#include <nuway_common/trajectory.h>

namespace nuway_control {
namespace {

using nuway_common::BicycleInput;
using nuway_common::BicycleInputJacobian;
using nuway_common::BicycleLinearization;
using nuway_common::BicycleParams;
using nuway_common::BicycleState;
using nuway_common::BicycleStateJacobian;
using nuway_common::kBicycleAccel;
using nuway_common::kBicycleDelta;
using nuway_common::kBicycleDeltaCmd;
using nuway_common::kBicyclePsi;
using nuway_common::kBicycleStateDim;
using nuway_common::kBicycleV;
using nuway_common::kBicycleX;
using nuway_common::kBicycleY;

using Index = Eigen::Index;
constexpr Index kNx = kBicycleStateDim;
constexpr Index kNu = nuway_common::kBicycleInputDim;

const char* StatusName(OsqpEigen::Status status) {
  switch (status) {
    case OsqpEigen::Status::Solved:
      return "solved";
    case OsqpEigen::Status::SolvedInaccurate:
      return "solved_inaccurate";
    case OsqpEigen::Status::MaxIterReached:
      return "max_iter_reached";
    case OsqpEigen::Status::PrimalInfeasible:
      return "primal_infeasible";
    case OsqpEigen::Status::PrimalInfeasibleInaccurate:
      return "primal_infeasible_inaccurate";
    case OsqpEigen::Status::DualInfeasible:
      return "dual_infeasible";
    case OsqpEigen::Status::DualInfeasibleInaccurate:
      return "dual_infeasible_inaccurate";
    case OsqpEigen::Status::NonCvx:
      return "non_convex";
    case OsqpEigen::Status::Sigint:
      return "sigint";
    case OsqpEigen::Status::Unsolved:
      return "unsolved";
    default:
      return "unknown";
  }
}

MpcSolveStatus Reduce(OsqpEigen::Status status) {
  switch (status) {
    case OsqpEigen::Status::Solved:
      return MpcSolveStatus::kSolved;
    case OsqpEigen::Status::SolvedInaccurate:
      return MpcSolveStatus::kSolvedInaccurate;
    case OsqpEigen::Status::MaxIterReached:
      return MpcSolveStatus::kMaxIterReached;
    default:
      return MpcSolveStatus::kOther;
  }
}

// One reference knot: the state and input the model is linearised about.
constexpr double kStopReferenceSpeedMps = 0.05;

struct Knot {
  BicycleState x = BicycleState::Zero();
  BicycleInput u = BicycleInput::Zero();
};

// Position error (map dx, dy) rotated into the knot's local frame: the
// longitudinal component along the reference heading, the lateral one to
// its left.
Eigen::Matrix2d LocalFrame(double yaw_ref) {
  const double c = std::cos(yaw_ref);
  const double s = std::sin(yaw_ref);
  Eigen::Matrix2d t;
  t << c, s, -s, c;
  return t;
}

}  // namespace

const char* MpcOutcomeName(MpcOutcome outcome) {
  switch (outcome) {
    case MpcOutcome::kSolved:
      return "solved";
    case MpcOutcome::kBudget:
      return "budget";
    case MpcOutcome::kNoIterate:
      return "no_iterate";
    case MpcOutcome::kNoInput:
      return "no_input";
  }
  return "unknown";
}

MpcOutcome ClassifySolve(MpcSolveStatus status, int polish_status,
                         bool finite) {
  // The polish status is not looked at: a failed polish leaves the ADMM
  // iterate, which converged to tolerance, and is not a failure (§3.8).
  (void)polish_status;
  if (!finite) {
    return MpcOutcome::kNoIterate;
  }
  switch (status) {
    case MpcSolveStatus::kSolved:
      return MpcOutcome::kSolved;
    case MpcSolveStatus::kSolvedInaccurate:
    case MpcSolveStatus::kMaxIterReached:
      return MpcOutcome::kBudget;
    case MpcSolveStatus::kOther:
      return MpcOutcome::kNoIterate;
  }
  return MpcOutcome::kNoIterate;
}

struct Mpc::Impl {
  VehicleModel model;
  MpcOptions options;
  Index n = 20;     // knots
  Index nu = 40;    // decision variables
  Index nc = 80;    // constraint rows
  int n_delay = 2;  // delay steps
  double delta_max = 1.0;

  // The QP data of the current tick; P and A are stored with every entry
  // of their pattern explicit so the pattern never changes between ticks
  // (osqp-eigen re-initialises the solver when it does).
  Eigen::SparseMatrix<double> hessian;
  Eigen::SparseMatrix<double> constraints;
  Eigen::VectorXd gradient;
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  Eigen::MatrixXd s_u;        // e = s_u u + s_const, 5n x 2n
  Eigen::VectorXd s_const;    // 5n
  Eigen::VectorXd rate_bias;  // 2n
  std::vector<Knot> knots;    // n + 1

  std::unique_ptr<OsqpEigen::Solver> solver;
  bool initialized = false;
  Eigen::VectorXd last_solution;
  bool have_solution = false;

  // Cross-tick state.
  struct Applied {
    BicycleInput u = BicycleInput::Zero();
    bool emergency_stop = false;
  };
  std::deque<Applied> commands;  // the last n_delay applied inputs
  double accel_bias = 0.0;       // a_off of the model
  std::optional<BicycleInput> u_prev;
  double delta_model = 0.0;
  int consecutive_failures = 0;
  double last_delta_ref = 0.0;

  // True when every sampled reference knot stands still (a stop at rest).
  bool ReferenceIsAStop() const {
    return std::all_of(knots.begin(), knots.end(), [](const Knot& knot) {
      return knot.x[kBicycleV] <= kStopReferenceSpeedMps;
    });
  }

  explicit Impl(VehicleModel m, MpcOptions o)
      : model(std::move(m)), options(o) {
    n = std::max<Index>(2, options.horizon);
    nu = kNu * n;
    nc = 2 * nu;
    n_delay = std::max(
        0, static_cast<int>(std::lround(options.t_delay_s / options.dt_s)));
    delta_max = model.max_steer_angle_rad;
    BuildConstraintMatrix();
    gradient.setZero(nu);
    lower.setZero(nc);
    upper.setZero(nc);
    s_u.setZero(kNx * n, nu);
    s_const.setZero(kNx * n);
    rate_bias.setZero(nu);
    knots.resize(static_cast<std::size_t>(n) + 1);
  }

  // Rows 0..2n-1 pick each input (box bounds), rows 2n..4n-1 the
  // difference u_k - u_{k-1} (rate bounds; k = 0 against u_prev through
  // the bounds).
  void BuildConstraintMatrix() {
    std::vector<Eigen::Triplet<double>> rows;
    for (Index i = 0; i < nu; ++i) {
      rows.emplace_back(i, i, 1.0);
      rows.emplace_back(nu + i, i, 1.0);
      if (i >= kNu) {
        rows.emplace_back(nu + i, i - kNu, -1.0);
      }
    }
    constraints.resize(nc, nu);
    constraints.setFromTriplets(rows.begin(), rows.end());
  }

  BicycleParams Params(double speed_mps) const {
    BicycleParams p;
    p.wheelbase_m = model.EffectiveWheelbaseM(std::max(0.0, speed_mps));
    p.tau_steer_s = std::max(1e-3, model.tau_steer_s);
    p.accel_offset_mps2 = accel_bias;
    return p;
  }

  double DeltaRef(double kappa, double wheelbase) const {
    return std::clamp(std::atan(kappa * wheelbase), -delta_max, delta_max);
  }

  // The reference knots at t_delay + k dt, k = 0..n.
  void SampleReference(const nuway_common::Trajectory& traj, double wheelbase) {
    for (Index k = 0; k <= n; ++k) {
      const nuway_common::TrajectoryPoint p = nuway_common::Interpolate(
          traj, options.t_delay_s + (static_cast<double>(k) * options.dt_s));
      Knot& knot = knots[static_cast<std::size_t>(k)];
      knot.x << p.x, p.y, p.yaw, std::max(0.0, p.v),
          DeltaRef(p.kappa, wheelbase);
      // The input that realises a_ref under the estimated disturbance.
      knot.u << p.a + accel_bias, knot.x[kBicycleDelta];
    }
  }

  // Rolls the measured state forward through the commands already issued
  // but not yet acting (the delay). Missing history (episode start) holds
  // the measured wheel angle at zero acceleration.
  BicycleState Compensate(const BicycleState& measured,
                          const BicycleParams& params) const {
    BicycleState x = measured;
    BicycleInput hold;
    hold << 0.0, measured[kBicycleDelta];
    const int have = static_cast<int>(commands.size());
    for (int i = 0; i < n_delay; ++i) {
      const int idx = i - (n_delay - have);
      const BicycleInput& u =
          idx >= 0 ? commands[static_cast<std::size_t>(idx)].u : hold;
      x = nuway_common::BicycleStepRk2(x, u, options.dt_s, params);
    }
    x[kBicyclePsi] = nuway_common::WrapAngle(x[kBicyclePsi]);
    return x;
  }

  // Linearises about the knots and condenses the error dynamics
  //   e_{k+1} = A_k e_k + B_k (u_k - u_ref_k) + d_k
  // into e = s_u u + s_const over k = 1..n (e_0 is fixed), then forms the
  // QP's P, q and bounds. Returns false when the data is not finite.
  bool BuildProblem(const BicycleState& x0, const BicycleParams& params,
                    const BicycleInput& u_prev_value) {
    BicycleState e0 = x0 - knots[0].x;
    e0[kBicyclePsi] = nuway_common::WrapAngle(e0[kBicyclePsi]);
    s_u.setZero();
    Eigen::VectorXd u_ref(nu);
    // Recursion over the blocks: block k+1 = A_k block k + [B_k at column
    // block k]; s_{k+1} = A_k s_k + d_k with s_0 = e_0.
    BicycleState s = e0;
    for (Index k = 0; k < n; ++k) {
      const Knot& knot = knots[static_cast<std::size_t>(k)];
      const BicycleLinearization lin = nuway_common::BicycleLinearizeRk2(
          knot.x, knot.u, options.dt_s, params);
      BicycleState d = lin.x_next - knots[static_cast<std::size_t>(k) + 1].x;
      d[kBicyclePsi] = nuway_common::WrapAngle(d[kBicyclePsi]);
      u_ref.segment<kNu>(kNu * k) = knot.u;
      const Index row = kNx * k;  // block of e_{k+1}
      if (k > 0) {
        s_u.block(row, 0, kNx, kNu * k) =
            lin.a * s_u.block(row - kNx, 0, kNx, kNu * k);
      }
      s_u.block<kNx, kNu>(row, kNu * k) = lin.b;
      s = (lin.a * s) + d;
      s_const.segment<kNx>(row) = s;
    }
    // e = s_u (u - u_ref) + s_const = s_u u + s_shift.
    const Eigen::VectorXd s_shift = s_const - (s_u * u_ref);
    // Q-bar: block k (for e_{k+1}) is T^T diag(q) T in the knot's frame.
    Eigen::MatrixXd q_bar = Eigen::MatrixXd::Zero(kNx * n, kNx * n);
    for (Index k = 0; k < n; ++k) {
      const Knot& knot = knots[static_cast<std::size_t>(k) + 1];
      const double scale = k == n - 1 ? options.q_terminal_scale : 1.0;
      const Eigen::Matrix2d t = LocalFrame(knot.x[kBicyclePsi]);
      Eigen::Matrix<double, kNx, kNx> qk =
          Eigen::Matrix<double, kNx, kNx>::Zero();
      qk.block<2, 2>(0, 0) =
          t.transpose() *
          Eigen::Vector2d(options.q[0], options.q[1]).asDiagonal() * t;
      qk(kBicyclePsi, kBicyclePsi) = options.q[2];
      qk(kBicycleV, kBicycleV) = options.q[3];
      qk(kBicycleDelta, kBicycleDelta) = options.q[4];
      q_bar.block<kNx, kNx>(kNx * k, kNx * k) = scale * qk;
    }
    // R-bar and the rate term D^T R_d D with D the difference operator
    // (rows 2n.. of the constraint matrix) and b = [u_prev; 0; ...].
    Eigen::VectorXd r_bar(nu);
    Eigen::VectorXd rd_bar(nu);
    for (Index k = 0; k < n; ++k) {
      r_bar.segment<kNu>(kNu * k) << options.r[0], options.r[1];
      rd_bar.segment<kNu>(kNu * k) << options.r_d[0], options.r_d[1];
    }
    const Eigen::MatrixXd d_op =
        Eigen::MatrixXd(constraints).block(nu, 0, nu, nu);
    // b = [u_prev; 0; ...]: the rate term's offset (a member so GCC's
    // null-dereference analysis does not chase a fresh allocation).
    rate_bias.setZero();
    rate_bias[kBicycleAccel] = u_prev_value[kBicycleAccel];
    rate_bias[kBicycleDeltaCmd] = u_prev_value[kBicycleDeltaCmd];
    const Eigen::MatrixXd h = (s_u.transpose() * q_bar * s_u) +
                              Eigen::MatrixXd(r_bar.asDiagonal()) +
                              (d_op.transpose() * rd_bar.asDiagonal() * d_op);
    const Eigen::VectorXd g =
        (s_u.transpose() * q_bar * s_shift) - (r_bar.asDiagonal() * u_ref) -
        (d_op.transpose() * (rd_bar.asDiagonal() * rate_bias));
    // OSQP minimises 1/2 u^T P u + q^T u.
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<std::size_t>(nu) *
                     (static_cast<std::size_t>(nu) + 1) / 2);
    for (Index i = 0; i < nu; ++i) {
      for (Index j = i; j < nu; ++j) {
        triplets.emplace_back(i, j, 2.0 * h(i, j));
      }
    }
    hessian.resize(nu, nu);
    hessian.setFromTriplets(triplets.begin(), triplets.end());
    gradient = 2.0 * g;
    // Bounds: the input boxes, then the rates (k = 0 against u_prev).
    const double da = model.limits.jerk_max_mps3 * options.dt_s;
    const double dd = model.limits.steer_rate_max_radps * options.dt_s;
    for (Index k = 0; k < n; ++k) {
      const Index i = kNu * k;
      lower[i] = model.limits.a_min_mps2;
      upper[i] = model.limits.a_max_mps2;
      lower[i + 1] = -delta_max;
      upper[i + 1] = delta_max;
      const double a0 = k == 0 ? u_prev_value[kBicycleAccel] : 0.0;
      const double d0 = k == 0 ? u_prev_value[kBicycleDeltaCmd] : 0.0;
      lower[nu + i] = a0 - da;
      upper[nu + i] = a0 + da;
      lower[nu + i + 1] = d0 - dd;
      upper[nu + i + 1] = d0 + dd;
    }
    return gradient.allFinite() && s_shift.allFinite() && h.allFinite();
  }

  bool Setup() {
    solver = std::make_unique<OsqpEigen::Solver>();
    const MpcQpSettings& s = options.qp;
    solver->settings()->setVerbosity(false);
    solver->settings()->setWarmStart(s.warm_start);
    solver->settings()->setAdaptiveRho(false);
    solver->settings()->setRho(s.rho);
    solver->settings()->setSigma(s.sigma);
    solver->settings()->setScaling(s.scaling);
    solver->settings()->setMaxIteration(s.max_iter);
    solver->settings()->setCheckTermination(s.check_termination);
    solver->settings()->setPolish(s.polish);
    solver->settings()->setAbsoluteTolerance(s.eps_abs);
    solver->settings()->setRelativeTolerance(s.eps_rel);
    solver->data()->setNumberOfVariables(static_cast<int>(nu));
    solver->data()->setNumberOfConstraints(static_cast<int>(nc));
    if (!solver->data()->setHessianMatrix(hessian) ||
        !solver->data()->setGradient(gradient) ||
        !solver->data()->setLinearConstraintsMatrix(constraints) ||
        !solver->data()->setLowerBound(lower) ||
        !solver->data()->setUpperBound(upper) || !solver->initSolver()) {
      solver.reset();
      initialized = false;
      return false;
    }
    initialized = true;
    return true;
  }

  // Loads this tick's data into a fresh workspace. Updating P, q and the
  // bounds in place (osqp_update_*) keeps the Ruiz equilibration computed
  // at setup, and the problem changes shape with speed (the position rows
  // of S_u scale with v): the scaling of a tick at rest left the solver
  // at 6 m/s exhausting its budget every few ticks in a limit cycle
  // (task 9's first drives). A dense 40-variable setup costs well under a
  // millisecond, so every tick gets its own equilibration; the warm start
  // survives in last_solution.
  bool Load() {
    solver.reset();
    initialized = false;
    return Setup();
  }

  // The previous solution shifted by one step (the last input repeated).
  void WarmStart() {
    if (!have_solution || !initialized) {
      return;
    }
    Eigen::VectorXd shifted = last_solution;
    shifted.head(nu - kNu) = last_solution.tail(nu - kNu);
    solver->setPrimalVariable(shifted);
  }

  BicycleInput Clamp(const BicycleInput& u,
                     const BicycleInput& u_prev_value) const {
    const double da = model.limits.jerk_max_mps3 * options.dt_s;
    const double dd = model.limits.steer_rate_max_radps * options.dt_s;
    BicycleInput out;
    out[kBicycleAccel] = std::clamp(
        std::clamp(u[kBicycleAccel], u_prev_value[kBicycleAccel] - da,
                   u_prev_value[kBicycleAccel] + da),
        model.limits.a_min_mps2, model.limits.a_max_mps2);
    out[kBicycleDeltaCmd] = std::clamp(
        std::clamp(u[kBicycleDeltaCmd], u_prev_value[kBicycleDeltaCmd] - dd,
                   u_prev_value[kBicycleDeltaCmd] + dd),
        -delta_max, delta_max);
    return out;
  }

  // Records the input the car actually receives this tick, for the delay
  // compensation, the rate bounds and the lag model. An emergency stop is
  // the adapter's full brake with the wheel at delta_ref: recorded as
  // a_min (the planning limit; the physical brake is stronger).
  // Updates a_off from the command that was acting during the last tick
  // (issued n_delay ticks ago) and the measured acceleration. Skipped
  // without a full history and for a measurement beyond twice the
  // vehicle's limits (the tick CARLA locks the wheels reports -23 m/s^2:
  // a physics artefact, not a disturbance). While the brake acts rather
  // than the pedal map -- after an emergency stop, and braking at a crawl
  // or at rest, where the simulator's brake is not the model's -- the
  // residual is unobservable and the estimate relaxes to the zero prior
  // with the same time constant: the brake-side residual learned while
  // stopping (CARLA under-brakes the map by about 2 m/s^2) must not carry
  // into the departure, where it would read as a throttle surplus and hold
  // the car at the line with the brake on.
  void UpdateAccelBias(double speed_mps, double accel_meas) {
    if (options.accel_bias_tau_s <= 0.0 ||
        static_cast<int>(commands.size()) < n_delay || n_delay == 0) {
      return;
    }
    const double plausible = 2.0 * std::max(std::abs(model.limits.a_min_mps2),
                                            std::abs(model.limits.a_max_mps2));
    if (std::abs(accel_meas) > plausible) {
      return;
    }
    const Applied& acting = commands.front();
    const bool brake_acting =
        acting.emergency_stop ||
        (std::abs(speed_mps) < options.accel_bias_stop_speed_mps &&
         acting.u[kBicycleAccel] <= 0.0);
    const double target =
        brake_acting ? 0.0 : acting.u[kBicycleAccel] - accel_meas;
    const double alpha = std::min(1.0, options.dt_s / options.accel_bias_tau_s);
    accel_bias += alpha * (target - accel_bias);
    accel_bias = std::clamp(accel_bias, -options.accel_bias_max_mps2,
                            options.accel_bias_max_mps2);
  }

  void Record(const BicycleInput& u, bool emergency_stop) {
    commands.push_back(Applied{u, emergency_stop});
    while (static_cast<int>(commands.size()) > n_delay) {
      commands.pop_front();
    }
    u_prev = u;
    const double r = options.dt_s / std::max(1e-3, model.tau_steer_s);
    const double mid =
        delta_model + (0.5 * r * (u[kBicycleDeltaCmd] - delta_model));
    delta_model += r * (u[kBicycleDeltaCmd] - mid);
  }

  // The predicted horizon from the solved inputs through the linear model.
  nuway_common::Trajectory Horizon(const BicycleState& x0,
                                   const Eigen::VectorXd& u,
                                   double wheelbase) const {
    const Eigen::VectorXd e = (s_u * u) + s_const - (s_u * [&] {
                                Eigen::VectorXd u_ref(nu);
                                for (Index k = 0; k < n; ++k) {
                                  u_ref.segment<kNu>(kNu * k) =
                                      knots[static_cast<std::size_t>(k)].u;
                                }
                                return u_ref;
                              }());
    nuway_common::Trajectory out;
    out.reserve(static_cast<std::size_t>(n) + 1);
    for (Index k = 0; k <= n; ++k) {
      const BicycleState x =
          k == 0 ? x0
                 : BicycleState(knots[static_cast<std::size_t>(k)].x +
                                e.segment<kNx>(kNx * (k - 1)));
      nuway_common::TrajectoryPoint p;
      p.t = options.t_delay_s + (static_cast<double>(k) * options.dt_s);
      p.x = x[kBicycleX];
      p.y = x[kBicycleY];
      p.yaw = nuway_common::WrapAngle(x[kBicyclePsi]);
      p.v = x[kBicycleV];
      p.a = k < n ? u[kNu * k] : u[nu - kNu];
      p.kappa = std::tan(x[kBicycleDelta]) / wheelbase;
      out.push_back(p);
    }
    return out;
  }
};

Mpc::Mpc(VehicleModel model, MpcOptions options)
    : impl_(std::make_unique<Impl>(std::move(model), options)) {}

Mpc::~Mpc() = default;
Mpc::Mpc(Mpc&&) noexcept = default;
Mpc& Mpc::operator=(Mpc&&) noexcept = default;

const MpcOptions& Mpc::options() const { return impl_->options; }
const VehicleModel& Mpc::model() const { return impl_->model; }

void Mpc::Reset() {
  Impl& im = *impl_;
  im.commands.clear();
  im.u_prev.reset();
  im.delta_model = 0.0;
  im.accel_bias = 0.0;
  im.consecutive_failures = 0;
  im.last_delta_ref = 0.0;
  im.have_solution = false;
  im.solver.reset();
  im.initialized = false;
}

void Mpc::set_max_iter(int max_iter) {
  Impl& im = *impl_;
  im.options.qp.max_iter = max_iter;
  // The live workspace copied its settings at setup; rebuild it on the
  // next tick (the warm start survives in last_solution).
  im.solver.reset();
  im.initialized = false;
}

MpcOutput Mpc::Step(const MpcInput& in) {
  Impl& im = *impl_;
  MpcOutput out;
  out.delta_ref_rad = im.last_delta_ref;
  out.consecutive_failures = im.consecutive_failures;
  const bool have_reference = in.trajectory != nullptr &&
                              !in.trajectory->empty() &&
                              !in.trajectory_degraded;
  if (!in.pose_valid || !have_reference) {
    out.outcome = MpcOutcome::kNoInput;
    if (!in.pose_valid) {
      out.status = "pose_invalid";
    } else if (in.trajectory_degraded) {
      out.status = "trajectory_degraded";
    } else {
      out.status = "no_trajectory";
    }
    out.emergency_stop = true;
    out.steering_angle_rad = im.last_delta_ref;
    BicycleInput applied;
    applied << im.model.limits.a_min_mps2, im.last_delta_ref;
    im.Record(applied, true);
    return out;
  }
  im.UpdateAccelBias(in.speed_mps, in.accel_mps2);
  out.accel_bias_mps2 = im.accel_bias;
  const BicycleParams params = im.Params(in.speed_mps);
  const double delta_meas =
      im.options.use_measured_steering ? in.steering_angle_rad : im.delta_model;
  BicycleState measured;
  measured << in.pose.x, in.pose.y, in.pose.yaw, in.speed_mps,
      std::clamp(delta_meas, -im.delta_max, im.delta_max);
  // Tracking errors against the reference of this tick (t = 0).
  {
    const nuway_common::TrajectoryPoint ref =
        nuway_common::Interpolate(*in.trajectory, 0.0);
    const Eigen::Vector2d local =
        LocalFrame(ref.yaw) *
        Eigen::Vector2d(in.pose.x - ref.x, in.pose.y - ref.y);
    out.lateral_error_m = local.y();
    out.heading_error_rad = nuway_common::WrapAngle(ref.yaw - in.pose.yaw);
    out.speed_error_mps = ref.v - in.speed_mps;
  }
  im.SampleReference(*in.trajectory, params.wheelbase_m);
  if (std::isfinite(im.knots[0].x[kBicycleDelta])) {
    im.last_delta_ref = im.knots[0].x[kBicycleDelta];
  }
  out.delta_ref_rad = im.last_delta_ref;
  const BicycleState x0 = im.Compensate(measured, params);
  BicycleInput u_prev_value;
  if (im.u_prev.has_value()) {
    u_prev_value = *im.u_prev;
  } else {
    u_prev_value << 0.0, measured[kBicycleDelta];
  }
  u_prev_value = im.Clamp(u_prev_value, u_prev_value);

  // Solve.
  Eigen::VectorXd solution;
  if (!im.BuildProblem(x0, params, u_prev_value)) {
    out.outcome = MpcOutcome::kNoIterate;
    out.status = "non_finite_problem";
  } else if (!im.Load()) {
    out.outcome = MpcOutcome::kNoIterate;
    out.status = "setup_failed";
  } else {
    im.WarmStart();
    const auto start = std::chrono::steady_clock::now();
    const OsqpEigen::ErrorExitFlag flag = im.solver->solveProblem();
    out.solve_time_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - start)
                            .count();
    if (flag != OsqpEigen::ErrorExitFlag::NoError) {
      out.outcome = MpcOutcome::kNoIterate;
      out.status = "solver_error";
    } else {
      const OsqpEigen::Status status = im.solver->getStatus();
      out.status = StatusName(status);
      out.iterations = static_cast<int>(im.solver->workspace()->info->iter);
      solution = im.solver->getSolution();
      out.outcome = ClassifySolve(
          Reduce(status),
          static_cast<int>(im.solver->workspace()->info->status_polish),
          solution.size() == im.nu && solution.allFinite());
    }
  }

  // The policy.
  if (out.outcome == MpcOutcome::kSolved) {
    im.consecutive_failures = 0;
  } else {
    ++im.consecutive_failures;
    out.counted_failure = true;
  }
  out.consecutive_failures = im.consecutive_failures;
  const bool escalate =
      out.outcome == MpcOutcome::kNoIterate ||
      im.consecutive_failures >= im.options.max_consecutive_solver_failures;
  if (out.outcome != MpcOutcome::kNoIterate) {
    im.last_solution = solution;
    im.have_solution = true;
    out.horizon = im.Horizon(x0, solution, params.wheelbase_m);
  }
  if (escalate) {
    out.emergency_stop = true;
    out.steering_angle_rad = im.last_delta_ref;
    out.solver_ok = false;
    BicycleInput applied;
    applied << im.model.limits.a_min_mps2, im.last_delta_ref;
    im.Record(applied, true);
    return out;
  }
  BicycleInput u0 = im.Clamp(solution.head<kNu>(), u_prev_value);
  // A stop reference (v_ref = 0 over the horizon) is never chased forward:
  // the QP would trade a heading or lateral error at rest for motion and
  // creep the car in a circle with the wheel turned (task 17 protocol).
  if (im.ReferenceIsAStop()) {
    u0[kBicycleAccel] = std::min(u0[kBicycleAccel], 0.0);
  }
  out.accel_mps2 = u0[kBicycleAccel];
  out.steering_angle_rad = u0[kBicycleDeltaCmd];
  out.emergency_stop = false;
  out.solver_ok = !out.counted_failure;
  im.Record(u0, false);
  return out;
}

}  // namespace nuway_control
