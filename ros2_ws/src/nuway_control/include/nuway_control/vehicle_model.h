// The ego vehicle model of configs/vehicle/<vehicle>.yaml (M0 §2.6):
// geometry measured from the CARLA actor, actuator limits, and the sysid fit
// (time constants, fitted wheelbase, understeer gradient, LongitudinalMap).
#ifndef NUWAY_CONTROL_VEHICLE_MODEL_H_
#define NUWAY_CONTROL_VEHICLE_MODEL_H_

#include <optional>
#include <string>

#include "nuway_control/longitudinal_map.h"

namespace nuway_control {

struct VehicleLimits {
  double a_max_mps2 = 3.0;
  double a_min_mps2 = -6.0;
  double jerk_max_mps3 = 5.0;
  double steer_rate_max_radps = 0.8;
  double kappa_max = 0.18;
};

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

  // Effective wheelbase of the steering geometry at speed v.
  double EffectiveWheelbaseM(double speed_mps) const {
    return wheelbase_fitted_m + (understeer_gradient * speed_mps * speed_mps);
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
