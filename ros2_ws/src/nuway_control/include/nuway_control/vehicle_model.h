// The ego vehicle model of configs/vehicle/<vehicle>.yaml (M0 §2.6):
// geometry measured from the CARLA actor, actuator limits, and the sysid fit
// (time constants, fitted wheelbase, understeer gradient, LongitudinalMap).
//
// YAML fields read here (ROS convention, SI units; a missing key keeps the
// default in the struct below):
//   name                 vehicle id, e.g. lincoln_mkz_2020
//   wheelbase            geometric rear-to-front axle distance, m
//   length, width        footprint = 2 * bounding_box.extent.{x, y}, m
//   rear_axle_offset_x   base_link (rear axle) relative to the CARLA actor
//                        origin along x, m (negative = behind the origin)
//   max_steer_angle      front wheel angle at |steer| = 1, rad
//   tau_steer,           first-order actuator lags from the step responses
//   tau_throttle         (time to 63 % of steady state), s; the M1 MPC model
//   wheelbase_fitted     bicycle-model wheelbase fitted from the steer
//                        sweeps (yaw_rate = v tan(delta) / L), m
//   understeer_gradient  K in yaw_rate (L + K v^2) = v tan(delta), s^2/m
//   limits: {a_max, a_min, jerk_max, steer_rate_max, kappa_max}
//                        bounds every controller clamps its command to
//   longitudinal_map     the sysid pedal tables (longitudinal_map.h)
// `height`, `bbox_center_z`, `actor_origin_height` and the `sysid:` record
// are read by the Python side (rig, collision footprint) and skipped here.
#ifndef NUWAY_CONTROL_VEHICLE_MODEL_H_
#define NUWAY_CONTROL_VEHICLE_MODEL_H_

#include <cmath>
#include <optional>
#include <string>

#include "nuway_control/longitudinal_map.h"

namespace nuway_control {

// The `limits:` block: what the controllers may command, not what the
// actuators can physically do (comfort and safety bounds).
struct VehicleLimits {
  double a_max_mps2 = 3.0;            // acceleration cap
  double a_min_mps2 = -6.0;           // braking cap (negative)
  double jerk_max_mps3 = 5.0;         // |da/dt| cap (used from M1)
  double steer_rate_max_radps = 0.8;  // |d delta / dt| cap
  // Comfort curvature from the YAML; unused by planning and control, which
  // bound curvature by VehicleModel::PhysicalCurvatureMax() (M1 §3.3).
  double kappa_max = 0.18;
};

// Everything the controllers know about the ego. Plain data: the node loads
// it once at start-up and hands it to the controller by value.
struct VehicleModel {
  std::string name;
  double wheelbase_m = 2.86;
  double length_m = 4.892;
  double width_m = 1.837;
  double rear_axle_offset_x_m = -1.389;  // actor origin -> rear axle
  double max_steer_angle_rad = 1.2217;
  double tau_steer_s = 0.1;
  double tau_throttle_s = 0.4;
  // Bicycle-model wheelbase that reproduces the measured yaw rate
  // (yaw_rate * (L + K v^2) = v tan(delta)); equals `wheelbase_m` with
  // K = 0 when the YAML carries no fit.
  double wheelbase_fitted_m = 2.86;
  double understeer_gradient = 0.0;
  VehicleLimits limits;
  std::optional<LongitudinalMap> longitudinal_map;

  // Effective wheelbase of the steering geometry at speed v: L + K v^2. A
  // real car turns less at speed than the kinematic tan(delta) / L predicts
  // (tyre slip angles grow with lateral load, i.e. understeer); folding
  // that into a longer wheelbase keeps the kinematic formulas valid, so the
  // pure pursuit steer for a wanted curvature kappa is atan(kappa L_eff).
  double EffectiveWheelbaseM(double speed_mps) const {
    return wheelbase_fitted_m + (understeer_gradient * speed_mps * speed_mps);
  }

  // The physical path-curvature limit of the kinematic bicycle at full
  // lock, tan(max_steer) / L (about 0.96 rad/m for the Lincoln): the bound
  // the M1 lattice filter and QP use, since the Town03 junction corners
  // (R = 2.4 m, kappa = 0.42) exceed the YAML's comfort limits.kappa_max.
  double PhysicalCurvatureMax() const {
    return std::tan(max_steer_angle_rad) / wheelbase_m;
  }
};

// Parses a vehicle YAML document; nullopt with `error` set on a malformed
// document or an invalid longitudinal map.
std::optional<VehicleModel> ParseVehicleModel(const std::string& text,
                                              std::string* error);

// Loads configs/vehicle/<vehicle>.yaml.
std::optional<VehicleModel> LoadVehicleModel(const std::string& path,
                                             std::string* error);

}  // namespace nuway_control

#endif  // NUWAY_CONTROL_VEHICLE_MODEL_H_
