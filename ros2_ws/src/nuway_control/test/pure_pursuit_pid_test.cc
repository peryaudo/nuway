#include "nuway_control/pure_pursuit_pid.h"

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <nuway_common/frenet.h>
#include <nuway_common/geometry.h>

namespace nuway_control {
namespace {

constexpr double kDt = 0.05;
constexpr double kPi = 3.14159265358979323846;

VehicleModel Lincoln() {
  std::string error;
  const std::optional<VehicleModel> model = LoadVehicleModel(
      std::string(NUWAY_REPO_ROOT) + "/configs/vehicle/lincoln_mkz_2020.yaml",
      &error);
  EXPECT_TRUE(model.has_value()) << error;
  return model.value_or(VehicleModel{});
}

// Straight line along +x from x0 to x1 sampled every 0.5 m.
nuway_common::ReferenceLine Straight(double x0, double x1) {
  nuway_common::Vector2dList points;
  const int n = static_cast<int>(std::lround((x1 - x0) / 0.5));
  for (int i = 0; i <= n; ++i) {
    points.emplace_back(x0 + (0.5 * i), 0.0);
  }
  return nuway_common::ReferenceLine::FromPoints(points);
}

// Counter-clockwise circle of the given radius centred at the origin,
// starting at angle 0.
nuway_common::ReferenceLine Circle(double radius) {
  nuway_common::Vector2dList points;
  const double step = 0.5 / radius;
  const int n = static_cast<int>(std::ceil(2.0 * kPi / step));
  for (int i = 0; i < n; ++i) {
    const double theta = step * i;
    points.emplace_back(radius * std::cos(theta), radius * std::sin(theta));
  }
  return nuway_common::ReferenceLine::FromPoints(points);
}

std::vector<double> Limit(const nuway_common::ReferenceLine& line,
                          double speed) {
  return std::vector<double>(static_cast<std::size_t>(line.size()), speed);
}

// Kinematic bicycle on the rear axle, driven by the commanded accel and
// wheel angle (no actuator lag).
struct Sim {
  nuway_common::SE2 pose;
  double speed = 0.0;
  void Step(const ControlOutput& out, double wheelbase) {
    const double accel = out.emergency_stop ? -6.0 : out.accel_mps2;
    const double steer = out.emergency_stop ? 0.0 : out.steering_angle_rad;
    speed = std::max(0.0, speed + (accel * kDt));
    pose.x += speed * std::cos(pose.yaw) * kDt;
    pose.y += speed * std::sin(pose.yaw) * kDt;
    pose.yaw = nuway_common::WrapAngle(
        pose.yaw + (speed / wheelbase * std::tan(steer) * kDt));
  }
};

TEST(PurePursuitPid, NoLineOrOffLineIsEmergencyStop) {
  PurePursuitPid controller(Lincoln(), PurePursuitPidOptions{});
  ControlOutput out = controller.Step({0.0, 0.0, 0.0}, 0.0, kDt);
  EXPECT_TRUE(out.emergency_stop);
  controller.SetReferenceLine(Straight(0.0, 100.0),
                              Limit(Straight(0.0, 100.0), 10.0));
  out = controller.Step({50.0, 8.0, 0.0}, 5.0, kDt);
  EXPECT_TRUE(out.emergency_stop);  // 8 m off, beyond max_lateral_error_m
  out = controller.Step({50.0, 1.0, 0.0}, 5.0, kDt);
  EXPECT_FALSE(out.emergency_stop);
  EXPECT_NEAR(out.lateral_error_m, 1.0, 1e-6);
  EXPECT_NEAR(out.s_m, 50.0, 1e-6);
  // Past the end: brake.
  out = controller.Step({100.5, 0.0, 0.0}, 5.0, kDt);
  EXPECT_TRUE(out.emergency_stop);
  controller.Reset();
  EXPECT_FALSE(controller.has_reference_line());
}

TEST(PurePursuitPid, LookaheadAndSteerLimits) {
  const VehicleModel model = Lincoln();
  const PurePursuitPidOptions options;
  PurePursuitPid controller(model, options);
  const nuway_common::ReferenceLine line = Straight(0.0, 200.0);
  controller.SetReferenceLine(line, Limit(line, 10.0));
  // Lookahead clamps at both ends.
  EXPECT_NEAR(controller.Step({10.0, 0.0, 0.0}, 0.0, kDt).lookahead_m, 2.5,
              1e-9);
  EXPECT_NEAR(controller.Step({10.0, 0.0, 0.0}, 10.0, kDt).lookahead_m, 5.5,
              1e-9);
  EXPECT_NEAR(controller.Step({10.0, 0.0, 0.0}, 50.0, kDt).lookahead_m, 20.0,
              1e-9);
  // A pose left of the line steers right (negative), rate-limited per step.
  controller.Reset();
  controller.SetReferenceLine(line, Limit(line, 10.0));
  const double max_step = model.limits.steer_rate_max_radps * kDt;
  ControlOutput out = controller.Step({10.0, 2.0, 0.0}, 5.0, kDt);
  EXPECT_LT(out.steering_angle_rad, 0.0);
  EXPECT_NEAR(out.steering_angle_rad, -max_step, 1e-9);
  out = controller.Step({10.0, 2.0, 0.0}, 5.0, kDt);
  EXPECT_NEAR(out.steering_angle_rad, -2.0 * max_step, 1e-9);
  // Heading error is the line heading minus the ego yaw.
  out = controller.Step({10.0, 0.0, 0.1}, 5.0, kDt);
  EXPECT_NEAR(out.heading_error_rad, -0.1, 1e-9);
}

TEST(PurePursuitPid, TargetSpeedFollowsLimitCurvatureAndEnd) {
  PurePursuitPid controller(Lincoln(), PurePursuitPidOptions{});
  const nuway_common::ReferenceLine circle = Circle(30.0);
  controller.SetReferenceLine(circle, Limit(circle, 20.0));
  // sqrt(a_lat_max / kappa) = sqrt(1.5 * 30) on a 30 m circle (Menger on the
  // 0.5 m polygon is within 0.1 % of 1/30), evaluated a quarter turn in:
  // FromPoints gives the first and last samples zero curvature.
  ControlOutput out = controller.Step({0.0, 30.0, kPi}, 5.0, kDt);
  EXPECT_NEAR(out.target_speed_mps, std::sqrt(45.0), 0.05);
  const nuway_common::ReferenceLine straight = Straight(0.0, 100.0);
  controller.SetReferenceLine(straight, Limit(straight, 8.33));
  out = controller.Step({10.0, 0.0, 0.0}, 5.0, kDt);
  EXPECT_NEAR(out.target_speed_mps, 8.33, 1e-9);
  // Stop profile: sqrt(2 * 1.5 * remaining).
  out = controller.Step({97.0, 0.0, 0.0}, 5.0, kDt);
  EXPECT_NEAR(out.target_speed_mps, 3.0, 1e-9);
  EXPECT_LT(out.accel_mps2, 0.0);
}

TEST(PurePursuitPid, SlowsBeforeABend) {
  PurePursuitPid controller(Lincoln(), PurePursuitPidOptions{});
  // 100 m straight, then a quarter circle of radius 15 m (v_curve 5.48 m/s).
  nuway_common::Vector2dList points;
  for (int i = 0; i <= 200; ++i) {
    points.emplace_back(0.5 * i, 0.0);
  }
  const double radius = 15.0;
  const int n_arc = static_cast<int>(std::lround(0.5 * kPi * radius / 0.5));
  for (int i = 1; i <= n_arc; ++i) {
    const double theta = 0.5 * i / radius;
    points.emplace_back(100.0 + (radius * std::sin(theta)),
                        radius - (radius * std::cos(theta)));
  }
  // A long straight after the bend keeps the end-of-line stop profile out.
  const Eigen::Vector2d arc_end = points.back();
  for (int i = 1; i <= 600; ++i) {
    points.emplace_back(arc_end.x(), arc_end.y() + (0.5 * i));
  }
  const nuway_common::ReferenceLine line =
      nuway_common::ReferenceLine::FromPoints(points);
  controller.SetReferenceLine(line, Limit(line, 20.0));
  const double v_curve = std::sqrt(1.5 * radius);  // a_lat_max 1.5
  // 50 m before the bend: allowed sqrt(v_curve^2 + 2 * 2 * 50).
  ControlOutput out = controller.Step({50.0, 0.0, 0.0}, 10.0, kDt);
  EXPECT_NEAR(out.target_speed_mps,
              std::sqrt((v_curve * v_curve) + (2.0 * 2.0 * 50.0)), 0.3);
  EXPECT_LT(out.target_speed_mps, 20.0);
  // 1 m before the bend (its first full-curvature sample is 1.5 m ahead):
  // the curvature speed plus that little braking distance.
  out = controller.Step({99.0, 0.0, 0.0}, 10.0, kDt);
  EXPECT_NEAR(out.target_speed_mps,
              std::sqrt((v_curve * v_curve) + (2.0 * 2.0 * 1.5)), 0.2);
  // Far away (beyond the 60 m horizon): the limit.
  out = controller.Step({20.0, 0.0, 0.0}, 10.0, kDt);
  EXPECT_NEAR(out.target_speed_mps, 20.0, 1e-9);
  // On the braking ramp at the profile speed the feed-forward alone commands
  // the ramp's deceleration (fresh controller: no integral or derivative).
  PurePursuitPid fresh(Lincoln(), PurePursuitPidOptions{});
  fresh.SetReferenceLine(line, Limit(line, 20.0));
  const double v_ramp =
      fresh.Step({50.0, 0.0, 0.0}, 10.0, kDt).target_speed_mps;
  fresh.Reset();
  fresh.SetReferenceLine(line, Limit(line, 20.0));
  out = fresh.Step({50.0, 0.0, 0.0}, v_ramp, kDt);  // primes the derivative
  out = fresh.Step({50.0, 0.0, 0.0}, v_ramp, kDt);
  EXPECT_NEAR(out.accel_mps2, -2.0, 0.6);
  // A speed-limit drop ahead is braked for the same way: 30 m before a 5 m/s
  // section, sqrt(5^2 + 2 * 2 * 30).
  std::vector<double> limits = Limit(line, 20.0);
  for (std::size_t i = 0; i < limits.size(); ++i) {
    if (line.s()[i] >= 60.0) {
      limits[i] = 5.0;
    }
  }
  controller.SetReferenceLine(line, limits);
  out = controller.Step({30.0, 0.0, 0.0}, 10.0, kDt);
  EXPECT_NEAR(out.target_speed_mps, std::sqrt(25.0 + (2.0 * 2.0 * 30.0)), 0.3);
}

TEST(PurePursuitPid, ConvergesOnAStraight) {
  const VehicleModel model = Lincoln();
  PurePursuitPid controller(model, PurePursuitPidOptions{});
  const nuway_common::ReferenceLine line = Straight(0.0, 400.0);
  controller.SetReferenceLine(line, Limit(line, 10.0));
  Sim sim{{5.0, 1.5, 0.2}, 0.0};
  double max_speed = 0.0;
  for (int k = 0; k < 400; ++k) {  // 20 s
    const ControlOutput out = controller.Step(sim.pose, sim.speed, kDt);
    ASSERT_FALSE(out.emergency_stop) << k;
    sim.Step(out, model.wheelbase_m);
    max_speed = std::max(max_speed, sim.speed);
  }
  EXPECT_NEAR(sim.speed, 10.0, 0.3);
  EXPECT_LT(max_speed, 11.5);  // no big overshoot
  EXPECT_LT(std::abs(sim.pose.y), 0.1);
  EXPECT_LT(std::abs(sim.pose.yaw), 0.02);
}

TEST(PurePursuitPid, TracksACircleAtTheCurvatureSpeed) {
  const VehicleModel model = Lincoln();
  PurePursuitPid controller(model, PurePursuitPidOptions{});
  const double radius = 30.0;
  const nuway_common::ReferenceLine circle = Circle(radius);
  controller.SetReferenceLine(circle, Limit(circle, 20.0));
  Sim sim{{radius, 0.0, kPi / 2.0}, 3.0};
  double max_lateral = 0.0;
  ControlOutput out;
  // 20 s: about 150 m of the 188 m line, before the stop profile at its end.
  for (int k = 0; k < 400; ++k) {
    out = controller.Step(sim.pose, sim.speed, kDt);
    ASSERT_FALSE(out.emergency_stop) << k;
    sim.Step(out, model.wheelbase_m);
    if (k > 200) {
      max_lateral = std::max(max_lateral, std::abs(out.lateral_error_m));
    }
  }
  EXPECT_NEAR(sim.speed, std::sqrt(1.5 * radius), 0.3);
  EXPECT_LT(max_lateral, 0.3);
  // Steady steer of the kinematic model on this radius: atan(L / R).
  EXPECT_NEAR(out.steering_angle_rad, std::atan(model.wheelbase_m / radius),
              0.03);
}

TEST(PurePursuitPid, StopsAtTheEndOfTheLine) {
  const VehicleModel model = Lincoln();
  PurePursuitPid controller(model, PurePursuitPidOptions{});
  const nuway_common::ReferenceLine line = Straight(0.0, 100.0);
  controller.SetReferenceLine(line, Limit(line, 15.0));
  Sim sim{{0.0, 0.0, 0.0}, 10.0};
  for (int k = 0; k < 600; ++k) {
    const ControlOutput out = controller.Step(sim.pose, sim.speed, kDt);
    sim.Step(out, model.wheelbase_m);
  }
  EXPECT_LT(sim.speed, 0.1);
  EXPECT_LT(sim.pose.x, 100.5);
  EXPECT_GT(sim.pose.x, 95.0);
}

}  // namespace
}  // namespace nuway_control
