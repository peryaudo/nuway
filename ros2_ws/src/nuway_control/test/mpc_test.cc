#include "nuway_control/mpc.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <nuway_common/bicycle_model.h>
#include <nuway_common/geometry.h>
#include <nuway_common/trajectory.h>

namespace nuway_control {
namespace {

using nuway_common::BicycleInput;
using nuway_common::BicycleState;
using nuway_common::SE2;
using nuway_common::Trajectory;
using nuway_common::TrajectoryPoint;

constexpr double kDt = 0.05;

VehicleModel Lincoln() {
  std::string error;
  const std::optional<VehicleModel> model = LoadVehicleModel(
      std::string(NUWAY_REPO_ROOT) + "/configs/vehicle/lincoln_mkz_2020.yaml",
      &error);
  EXPECT_TRUE(model.has_value()) << error;
  return model.value_or(VehicleModel{});
}

// A map-frame path parameterised by time; Sample(t0) is the trajectory a
// producer would publish at time t0 (81 points at 0.1 s, t relative).
struct Path {
  double v = 10.0;
  double radius = std::numeric_limits<double>::infinity();  // inf: straight
  double accel = 0.0;
  double x0 = 0.0;  // start of a straight path

  TrajectoryPoint At(double t) const {
    TrajectoryPoint p;
    p.t = t;
    p.v = std::max(0.0, v + (accel * t));
    p.a = p.v > 0.0 ? accel : 0.0;
    const double s = (v * t) + (0.5 * accel * t * t);
    if (std::isinf(radius)) {
      p.x = x0 + s;
      p.y = 0.0;
      p.yaw = 0.0;
      p.kappa = 0.0;
    } else {
      const double theta = s / radius;
      p.x = radius * std::sin(theta);
      p.y = radius * (1.0 - std::cos(theta));
      p.yaw = nuway_common::WrapAngle(theta);
      p.kappa = 1.0 / radius;
    }
    return p;
  }

  Trajectory Sample(double t0) const {
    Trajectory out;
    for (int i = 0; i < nuway_common::kTrajectoryPoints; ++i) {
      TrajectoryPoint p = At(t0 + (i * nuway_common::kTrajectoryDtS));
      p.t = i * nuway_common::kTrajectoryDtS;
      out.push_back(p);
    }
    return out;
  }
};

// The plant: the same bicycle with steering lag, stepped at the tick with
// the command applied `delay` ticks later (the actuation delay the MPC
// compensates). An emergency stop is brake at a_min with the wheel at the
// commanded angle, as control_adapter does.
struct Plant {
  BicycleState x = BicycleState::Zero();
  nuway_common::BicycleParams params;  // accel_offset_mps2: a road grade
  int delay = 2;
  std::vector<BicycleInput> queue;  // pending commands, oldest first
  double ax = 0.0;                  // measured accel of the last tick

  void Apply(const MpcOutput& out, double a_min) {
    BicycleInput u;
    u << (out.emergency_stop ? a_min : out.accel_mps2), out.steering_angle_rad;
    queue.push_back(u);
    BicycleInput acting = BicycleInput::Zero();
    acting[1] = x[nuway_common::kBicycleDelta];
    if (static_cast<int>(queue.size()) > delay) {
      acting = queue.front();
      queue.erase(queue.begin());
    }
    const double v_before = x[nuway_common::kBicycleV];
    x = nuway_common::BicycleStepRk2(x, acting, kDt, params);
    x[nuway_common::kBicycleV] = std::max(0.0, x[nuway_common::kBicycleV]);
    ax = (x[nuway_common::kBicycleV] - v_before) / kDt;
    x[nuway_common::kBicyclePsi] =
        nuway_common::WrapAngle(x[nuway_common::kBicyclePsi]);
  }

  MpcInput Input(const Trajectory* traj) const {
    MpcInput in;
    in.pose = SE2{x[0], x[1], x[2]};
    in.speed_mps = x[3];
    in.accel_mps2 = ax;
    in.steering_angle_rad = x[4];
    in.trajectory = traj;
    return in;
  }
};

struct RunStats {
  double lat_abs_max = 0.0;
  double lat_abs_end = 0.0;
  double speed_err_end = 0.0;
  int max_iterations = 0;
  double mean_iterations = 0.0;
  double max_solve_ms = 0.0;
  int failures = 0;
  int stops = 0;
};

// Closes the loop for `ticks` ticks; the plant's delay equals the MPC's.
RunStats Drive(Mpc* mpc, Plant* plant, const Path& path, int ticks) {
  RunStats st;
  double sum_iter = 0.0;
  for (int k = 0; k < ticks; ++k) {
    const Trajectory traj = path.Sample(k * kDt);
    const MpcOutput out = mpc->Step(plant->Input(&traj));
    st.lat_abs_max = std::max(st.lat_abs_max, std::abs(out.lateral_error_m));
    st.lat_abs_end = std::abs(out.lateral_error_m);
    st.speed_err_end = out.speed_error_mps;
    st.max_iterations = std::max(st.max_iterations, out.iterations);
    sum_iter += out.iterations;
    st.max_solve_ms = std::max(st.max_solve_ms, out.solve_time_ms);
    st.failures += out.counted_failure ? 1 : 0;
    st.stops += out.emergency_stop ? 1 : 0;
    plant->Apply(out, mpc->model().limits.a_min_mps2);
  }
  st.mean_iterations = sum_iter / ticks;
  return st;
}

Plant PlantFor(const VehicleModel& model, const Path& path) {
  Plant plant;
  plant.params.wheelbase_m = model.wheelbase_fitted_m;
  plant.params.tau_steer_s = model.tau_steer_s;
  const TrajectoryPoint p0 = path.At(0.0);
  plant.x << p0.x, p0.y, p0.yaw, p0.v,
      std::atan(p0.kappa * plant.params.wheelbase_m);
  return plant;
}

TEST(MpcTest, TracksAStraightFromALateralOffset) {
  const VehicleModel model = Lincoln();
  Mpc mpc(model, MpcOptions{});
  Path path;
  path.v = 10.0;
  Plant plant = PlantFor(model, path);
  plant.x[1] = 0.5;  // half a metre left of the lane
  const RunStats st = Drive(&mpc, &plant, path, 120);
  EXPECT_EQ(st.failures, 0);
  EXPECT_EQ(st.stops, 0);
  EXPECT_LT(st.lat_abs_end, 0.03) << st.lat_abs_max;
  EXPECT_LT(std::abs(st.speed_err_end), 0.1);
  EXPECT_LT(st.lat_abs_max, 0.6);  // no overshoot past the start offset
  std::printf("straight: iterations max %d mean %.1f, solve max %.3f ms\n",
              st.max_iterations, st.mean_iterations, st.max_solve_ms);
}

TEST(MpcTest, TracksACircleAndABrakingProfile) {
  const VehicleModel model = Lincoln();
  Mpc mpc(model, MpcOptions{});
  Path path;
  path.v = 8.0;
  path.radius = 30.0;
  Plant plant = PlantFor(model, path);
  RunStats st = Drive(&mpc, &plant, path, 200);
  EXPECT_EQ(st.failures, 0);
  EXPECT_LT(st.lat_abs_max, 0.15);
  EXPECT_LT(std::abs(st.speed_err_end), 0.15);
  std::printf("circle: iterations max %d mean %.1f, solve max %.3f ms\n",
              st.max_iterations, st.mean_iterations, st.max_solve_ms);
  // Braking to rest at 2 m/s^2 on the same circle: the speed follows.
  Path braking = path;
  braking.accel = -2.0;
  Plant plant2 = PlantFor(model, braking);
  Mpc mpc2(model, MpcOptions{});
  st = Drive(&mpc2, &plant2, braking, 100);  // at rest after 4 s
  EXPECT_EQ(st.failures, 0);
  EXPECT_LT(std::abs(st.speed_err_end), 0.2);
  EXPECT_LT(plant2.x[3], 0.3);
  EXPECT_LT(st.lat_abs_max, 0.2);
}

TEST(MpcTest, GradeIsCompensatedByTheDisturbanceObserver) {
  const VehicleModel model = Lincoln();
  Path path;
  path.v = 8.0;
  // A 15 % climb (the Town03 dev route start): 1.5 m/s^2 against the car.
  Plant plant = PlantFor(model, path);
  plant.params.accel_offset_mps2 = 1.5;
  Mpc mpc(model, MpcOptions{});
  const RunStats st = Drive(&mpc, &plant, path, 200);
  EXPECT_EQ(st.failures, 0);
  EXPECT_LT(std::abs(st.speed_err_end), 0.15) << st.speed_err_end;
  const Trajectory traj = path.Sample(200 * kDt);
  const MpcOutput out = mpc.Step(plant.Input(&traj));
  EXPECT_NEAR(out.accel_bias_mps2, 1.5, 0.15);
  EXPECT_NEAR(out.accel_mps2, 1.5, 0.3);  // holding speed on the grade
  // The tick CARLA locks the wheels reports EgoState.ax of -23 m/s^2: a
  // measurement beyond twice the limits does not move the estimate.
  MpcInput spike = plant.Input(&traj);
  spike.accel_mps2 = -23.0;
  EXPECT_NEAR(mpc.Step(spike).accel_bias_mps2, out.accel_bias_mps2, 1e-9);
  // A reset forgets the estimate.
  mpc.Reset();
  EXPECT_NEAR(mpc.Step(plant.Input(&traj)).accel_bias_mps2, 0.0, 1e-12);
  // The brake-side residual (CARLA brakes about 2 m/s^2 less than the M0
  // map) is learned while stopping but relaxes to zero once the brake
  // holds the car at rest: carried into the departure it would read as a
  // throttle surplus and keep the car at the line with the brake on.
  Plant weak = PlantFor(model, path);
  weak.params.accel_offset_mps2 = -2.0;
  Mpc other(model, MpcOptions{});
  Drive(&other, &weak, path, 200);
  const Trajectory ahead = path.Sample(200 * kDt);
  EXPECT_NEAR(other.Step(weak.Input(&ahead)).accel_bias_mps2, -2.0, 0.15);
  Trajectory halt = ahead;
  for (TrajectoryPoint& tp : halt) {
    tp.x = ahead[0].x;
    tp.y = ahead[0].y;
    tp.yaw = ahead[0].yaw;
    tp.v = 0.0;
    tp.a = -1.5;
  }
  MpcInput rest = weak.Input(&halt);
  rest.pose = SE2{ahead[0].x, ahead[0].y, ahead[0].yaw};
  rest.speed_mps = 0.0;
  rest.accel_mps2 = 0.0;
  for (int k = 0; k < 60; ++k) {
    EXPECT_LE(other.Step(rest).accel_mps2, 0.0);
  }
  EXPECT_NEAR(other.Step(rest).accel_bias_mps2, 0.0, 0.1);
}

// The stack's planner replans from the measured state every planning
// tick, so a speed deficit never accumulates into a tracking error the
// QP could see: on a grade the feedback alone lets the car slow to a
// stop (what happened on the ramp before the observer existed), the
// observer's feed-forward keeps it accelerating as planned.
TEST(MpcTest, ReplannedReferenceNeedsTheObserverOnAGrade) {
  const VehicleModel model = Lincoln();
  const auto climb = [&](MpcOptions options) {
    Mpc mpc(model, options);
    Plant plant;
    plant.params.wheelbase_m = model.wheelbase_fitted_m;
    plant.params.tau_steer_s = model.tau_steer_s;
    plant.params.accel_offset_mps2 = 1.5;
    plant.x << 0.0, 0.0, 0.0, 2.0, 0.0;
    for (int k = 0; k < 100; ++k) {
      Path path;  // the plan: accelerate at 0.5 from the current state
      path.x0 = plant.x[0];
      path.v = plant.x[3];
      path.accel = 0.5;
      const Trajectory traj = path.Sample(0.0);
      const MpcOutput out = mpc.Step(plant.Input(&traj));
      EXPECT_FALSE(out.emergency_stop);
      plant.Apply(out, model.limits.a_min_mps2);
    }
    return plant.x[3];
  };
  MpcOptions off;
  off.accel_bias_tau_s = 0.0;
  EXPECT_LT(climb(off), 1.0);
  EXPECT_GT(climb(MpcOptions{}), 3.0);  // ideal: 2 + 0.5 * 5 = 4.5
}

TEST(MpcTest, DelayCompensationRollsTheMeasuredStateForward) {
  const VehicleModel model = Lincoln();
  Mpc mpc(model, MpcOptions{});
  Path path;
  path.v = 6.0;
  path.radius = 40.0;
  Plant plant = PlantFor(model, path);
  std::vector<BicycleInput> issued;
  for (int k = 0; k < 3; ++k) {
    const Trajectory traj = path.Sample(k * kDt);
    const MpcInput in = plant.Input(&traj);
    const MpcOutput out = mpc.Step(in);
    ASSERT_FALSE(out.emergency_stop);
    ASSERT_EQ(out.horizon.size(), 21U);
    if (k == 2) {
      // horizon[0] is the measured state stepped through the two commands
      // issued on ticks 0 and 1 (still in flight), tau and L of the model.
      nuway_common::BicycleParams p;
      p.wheelbase_m = model.EffectiveWheelbaseM(in.speed_mps);
      p.tau_steer_s = model.tau_steer_s;
      p.accel_offset_mps2 = out.accel_bias_mps2;
      BicycleState x;
      x << in.pose.x, in.pose.y, in.pose.yaw, in.speed_mps,
          in.steering_angle_rad;
      x = nuway_common::BicycleStepRk2(x, issued[0], kDt, p);
      x = nuway_common::BicycleStepRk2(x, issued[1], kDt, p);
      EXPECT_NEAR(out.horizon[0].x, x[0], 1e-9);
      EXPECT_NEAR(out.horizon[0].y, x[1], 1e-9);
      EXPECT_NEAR(out.horizon[0].yaw, x[2], 1e-9);
      EXPECT_NEAR(out.horizon[0].v, x[3], 1e-9);
      EXPECT_NEAR(out.horizon[0].t, 0.1, 1e-12);
      EXPECT_NEAR(out.horizon.back().t, 1.1, 1e-12);
    }
    BicycleInput u;
    u << out.accel_mps2, out.steering_angle_rad;
    issued.push_back(u);
    plant.Apply(out, model.limits.a_min_mps2);
  }
}

TEST(MpcTest, PolishFailureIsNotASolverFailure) {
  EXPECT_EQ(ClassifySolve(MpcSolveStatus::kSolved, -1, true),
            MpcOutcome::kSolved);
  EXPECT_EQ(ClassifySolve(MpcSolveStatus::kSolved, 0, true),
            MpcOutcome::kSolved);
  EXPECT_EQ(ClassifySolve(MpcSolveStatus::kSolvedInaccurate, 1, true),
            MpcOutcome::kBudget);
  EXPECT_EQ(ClassifySolve(MpcSolveStatus::kMaxIterReached, 0, true),
            MpcOutcome::kBudget);
  EXPECT_EQ(ClassifySolve(MpcSolveStatus::kOther, 1, true),
            MpcOutcome::kNoIterate);
  EXPECT_EQ(ClassifySolve(MpcSolveStatus::kSolved, 1, false),
            MpcOutcome::kNoIterate);
  EXPECT_STREQ(MpcOutcomeName(MpcOutcome::kBudget), "budget");
}

TEST(MpcTest, ExhaustedBudgetUsesTheClampedIterateThenEscalates) {
  const VehicleModel model = Lincoln();
  Mpc mpc(model, MpcOptions{});
  Path path;
  path.v = 10.0;
  Plant plant = PlantFor(model, path);
  plant.x[1] = 1.0;
  mpc.set_max_iter(1);
  const double da_up = model.limits.jerk_max_mps3 * kDt;
  const double da_down = model.limits.jerk_brake_max_mps3 * kDt;
  const double dd = model.limits.steer_rate_max_radps * kDt;
  double prev_a = 0.0;
  double prev_d = plant.x[4];
  for (int k = 0; k < 4; ++k) {
    const Trajectory traj = path.Sample(k * kDt);
    const MpcOutput out = mpc.Step(plant.Input(&traj));
    EXPECT_EQ(out.outcome, MpcOutcome::kBudget) << out.status;
    EXPECT_TRUE(out.counted_failure);
    EXPECT_FALSE(out.solver_ok);
    EXPECT_FALSE(out.emergency_stop);
    EXPECT_EQ(out.consecutive_failures, k + 1);
    // Clamped to the boxes and to the rate limits against the last command.
    EXPECT_LE(out.accel_mps2, model.limits.a_max_mps2 + 1e-12);
    EXPECT_GE(out.accel_mps2, model.limits.a_min_mps2 - 1e-12);
    EXPECT_LE(out.accel_mps2 - prev_a, da_up + 1e-12);
    EXPECT_GE(out.accel_mps2 - prev_a, -da_down - 1e-12);
    EXPECT_LE(std::abs(out.steering_angle_rad - prev_d), dd + 1e-12);
    EXPECT_LE(std::abs(out.steering_angle_rad), model.max_steer_angle_rad);
    prev_a = out.accel_mps2;
    prev_d = out.steering_angle_rad;
    plant.Apply(out, model.limits.a_min_mps2);
  }
  // The fifth consecutive failure is an emergency stop at delta_ref.
  Trajectory traj = path.Sample(4 * kDt);
  MpcOutput out = mpc.Step(plant.Input(&traj));
  EXPECT_EQ(out.consecutive_failures, 5);
  EXPECT_TRUE(out.emergency_stop);
  EXPECT_FALSE(out.solver_ok);
  EXPECT_NEAR(out.steering_angle_rad, out.delta_ref_rad, 1e-12);
  plant.Apply(out, model.limits.a_min_mps2);
  // A solved tick resets the counter and the car drives again.
  mpc.set_max_iter(400);
  traj = path.Sample(5 * kDt);
  out = mpc.Step(plant.Input(&traj));
  EXPECT_EQ(out.outcome, MpcOutcome::kSolved) << out.status;
  EXPECT_EQ(out.consecutive_failures, 0);
  EXPECT_FALSE(out.emergency_stop);
  EXPECT_TRUE(out.solver_ok);
  // Reset clears the counter too.
  mpc.set_max_iter(1);
  traj = path.Sample(6 * kDt);
  EXPECT_EQ(mpc.Step(plant.Input(&traj)).consecutive_failures, 1);
  mpc.Reset();
  traj = path.Sample(7 * kDt);
  EXPECT_EQ(mpc.Step(plant.Input(&traj)).consecutive_failures, 1);
}

TEST(MpcTest, EmergencyStopsCarryDeltaRef) {
  const VehicleModel model = Lincoln();
  Mpc mpc(model, MpcOptions{});
  Path path;
  path.v = 5.0;
  path.radius = 12.0;
  const Plant plant = PlantFor(model, path);
  const Trajectory traj = path.Sample(0.0);
  const MpcOutput solved = mpc.Step(plant.Input(&traj));
  ASSERT_FALSE(solved.emergency_stop);
  const double delta_ref = std::atan(model.EffectiveWheelbaseM(5.0) / 12.0);
  EXPECT_NEAR(solved.delta_ref_rad, delta_ref, 1e-9);
  // Invalid pose: stop, wheel at the delta_ref last tracked.
  MpcInput in = plant.Input(&traj);
  in.pose_valid = false;
  MpcOutput out = mpc.Step(in);
  EXPECT_TRUE(out.emergency_stop);
  EXPECT_EQ(out.outcome, MpcOutcome::kNoInput);
  EXPECT_FALSE(out.counted_failure);
  EXPECT_NEAR(out.steering_angle_rad, delta_ref, 1e-9);
  // Degraded or missing trajectory: the same.
  in = plant.Input(&traj);
  in.trajectory_degraded = true;
  out = mpc.Step(in);
  EXPECT_TRUE(out.emergency_stop);
  EXPECT_NEAR(out.steering_angle_rad, delta_ref, 1e-9);
  in.trajectory = nullptr;
  in.trajectory_degraded = false;
  EXPECT_TRUE(mpc.Step(in).emergency_stop);
  // No usable iterate (a non-finite reference): stop at once, counted.
  Trajectory broken = traj;
  broken[10].x = std::numeric_limits<double>::quiet_NaN();
  out = mpc.Step(plant.Input(&broken));
  EXPECT_EQ(out.outcome, MpcOutcome::kNoIterate);
  EXPECT_TRUE(out.emergency_stop);
  EXPECT_TRUE(out.counted_failure);
  EXPECT_NEAR(out.steering_angle_rad, delta_ref, 1e-9);
  // A stop trajectory ("none") at rest is tracked: no emergency stop.
  Mpc fresh(model, MpcOptions{});
  Plant still;
  still.x << 3.0, -2.0, 0.4, 0.0, 0.0;
  const Trajectory none = nuway_common::StopTrajectory(SE2{3.0, -2.0, 0.4});
  out = fresh.Step(still.Input(&none));
  EXPECT_EQ(out.outcome, MpcOutcome::kSolved) << out.status;
  EXPECT_FALSE(out.emergency_stop);
  EXPECT_NEAR(out.accel_mps2, 0.0, 1e-3);
  EXPECT_NEAR(out.steering_angle_rad, 0.0, 1e-3);
}

TEST(MpcTest, LagModelReplacesTheMeasuredWheelAngle) {
  const VehicleModel model = Lincoln();
  MpcOptions options;
  options.use_measured_steering = false;
  Mpc mpc(model, options);
  Path path;
  path.v = 8.0;
  path.radius = 30.0;
  Plant plant = PlantFor(model, path);
  plant.x[4] = 0.0;  // the plant starts with a straight wheel too
  const RunStats st = Drive(&mpc, &plant, path, 200);
  EXPECT_EQ(st.failures, 0);
  EXPECT_LT(st.lat_abs_max, 0.3);
  EXPECT_LT(st.lat_abs_end, 0.15);
}

}  // namespace
}  // namespace nuway_control

namespace nuway_control {
namespace {

// The stop cases the first drives spent most ticks in: holding a "none"
// stop trajectory at rest (with a wheel angle off centre and a grade
// under the car) and rolling back into it. Every one must converge well
// inside the budget; the iteration counts are printed for the tuning note.
TEST(MpcTest, StopTrackingConvergesWithinBudget) {
  const VehicleModel model = Lincoln();
  struct Case {
    const char* name;
    double v;
    double steer;
    double grade;
    double yaw_off;
  };
  const std::vector<Case> cases = {
      {"rest", 0.0, 0.0, 0.0, 0.0},       {"rest_steer", 0.0, 0.4, 0.0, 0.0},
      {"rest_grade", 0.0, 0.0, 1.5, 0.0}, {"rolling_back", -0.9, 0.1, 1.5, 0.0},
      {"rest_yaw", 0.0, 0.0, 0.0, 0.3},   {"creep", 0.3, -0.2, -1.0, 0.1},
  };
  for (const Case& c : cases) {
    Mpc mpc(model, MpcOptions{});
    Plant plant;
    plant.params.wheelbase_m = model.wheelbase_fitted_m;
    plant.params.tau_steer_s = model.tau_steer_s;
    plant.params.accel_offset_mps2 = c.grade;
    plant.x << 5.0, -3.0, 0.7, c.v, c.steer;
    const Trajectory none =
        nuway_common::StopTrajectory(SE2{5.0, -3.0, 0.7 + c.yaw_off});
    int worst = 0;
    int failures = 0;
    for (int k = 0; k < 100; ++k) {
      MpcInput in = plant.Input(&none);
      in.speed_mps = plant.x[3];  // the plant clamps v at 0; keep the roll
      const MpcOutput out = mpc.Step(in);
      worst = std::max(worst, out.iterations);
      failures += out.counted_failure ? 1 : 0;
      plant.Apply(out, model.limits.a_min_mps2);
    }
    std::printf("%s: worst %d iterations, %d failures\n", c.name, worst,
                failures);
    EXPECT_EQ(failures, 0) << c.name;
    EXPECT_LT(worst, 200) << c.name;
  }
}

TEST(MpcTest, AStopReferenceIsNeverChasedForward) {
  // At rest with a 0.3 rad heading error against a stop at the same spot,
  // the QP would accelerate to turn the car toward the reference heading;
  // the command is clamped at zero so the car stays put (and the same from
  // a creep of 0.3 m/s: braking only).
  const VehicleModel model = Lincoln();
  for (const double v0 : {0.0, 0.3}) {
    Mpc mpc(model, MpcOptions{});
    Plant plant;
    plant.params.wheelbase_m = model.wheelbase_fitted_m;
    plant.params.tau_steer_s = model.tau_steer_s;
    plant.x << 5.0, -3.0, 0.7, v0, 0.0;
    const Trajectory stop =
        nuway_common::StopTrajectory(SE2{5.0, -3.0, 0.7 + 0.3});
    for (int k = 0; k < 60; ++k) {
      MpcInput in = plant.Input(&stop);
      in.speed_mps = plant.x[3];
      const MpcOutput out = mpc.Step(in);
      EXPECT_LE(out.accel_mps2, 1e-9) << "v0 " << v0 << " tick " << k;
      EXPECT_FALSE(out.emergency_stop);
      plant.Apply(out, model.limits.a_min_mps2);
    }
    EXPECT_LE(plant.x[3], v0 + 1e-6);
    EXPECT_NEAR(plant.x[0], 5.0, 0.5);
    EXPECT_NEAR(plant.x[1], -3.0, 0.5);
  }
}

}  // namespace

// A stop reference behind a hard-braking lead: the acceleration command
// falls at the brake build-up jerk (15 m/s^3 in the Lincoln YAML: a_min in
// 0.4 s) and never rises faster than the comfort jerk. With the symmetric
// 5 m/s^3 of the first draft the ramp alone took 1.2 s, most of the stopping
// distance at town speed (M1 task 17).
TEST(MpcTest, BrakingBuildsUpAtTheBrakeJerkAndReleasesAtTheComfortJerk) {
  const VehicleModel model = Lincoln();
  ASSERT_GT(model.limits.jerk_brake_max_mps3, model.limits.jerk_max_mps3);
  Mpc mpc(model, MpcOptions{});
  Path cruise;
  cruise.v = 8.0;
  Plant plant = PlantFor(model, cruise);
  // Settle on the cruise first so the previous command is ~0.
  Drive(&mpc, &plant, cruise, 40);
  Path stop;
  stop.v = 8.0;
  stop.accel = model.limits.a_min_mps2;
  stop.x0 = plant.x[0];
  const double da_up = model.limits.jerk_max_mps3 * kDt;
  const double da_down = model.limits.jerk_brake_max_mps3 * kDt;
  double prev_a = 0.0;
  double a_after_half_second = 0.0;
  for (int k = 0; k < 20; ++k) {
    const Trajectory traj = stop.Sample(k * kDt);
    const MpcOutput out = mpc.Step(plant.Input(&traj));
    ASSERT_TRUE(out.solver_ok) << out.status;
    EXPECT_LE(out.accel_mps2 - prev_a, da_up + 1e-9);
    EXPECT_GE(out.accel_mps2 - prev_a, -da_down - 1e-9);
    prev_a = out.accel_mps2;
    if (k == 9) {
      a_after_half_second = out.accel_mps2;
    }
    plant.Apply(out, model.limits.a_min_mps2);
  }
  // Ten ticks in, the command is already at (or near) full braking; the
  // comfort jerk would have allowed no more than -2.5 m/s^2 by then.
  EXPECT_LT(a_after_half_second, -4.0);
}

}  // namespace nuway_control
