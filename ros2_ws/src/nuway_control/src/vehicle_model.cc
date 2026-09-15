// VehicleModel: the yaml-cpp reader for configs/vehicle/<vehicle>.yaml. The
// field list and units are documented in the header.
#include "nuway_control/vehicle_model.h"

#include <fstream>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace nuway_control {
namespace {

// node[key] as T, or `fallback` when the key is absent (a present key of the
// wrong type still throws, which ParseVehicleModel turns into an error).
template <typename T>
T Get(const YAML::Node& node, const char* key, const T& fallback) {
  const YAML::Node value = node[key];
  return value.IsDefined() ? value.as<T>() : fallback;
}

}  // namespace

// Reads the scalar fields and the `limits:` block with the struct defaults
// as fallbacks, then the longitudinal map through its own reader. A YAML
// without a sysid fit gets wheelbase_fitted = wheelbase and K = 0, i.e. the
// plain kinematic bicycle; a missing or inconsistent longitudinal_map is an
// error, because the controller cannot bound its output without it.
std::optional<VehicleModel> ParseVehicleModel(const std::string& text,
                                              std::string* error) {
  VehicleModel model;
  // yaml-cpp reports malformed documents by throwing (see
  // longitudinal_map.cc).
  try {
    const YAML::Node root = YAML::Load(text);
    if (!root.IsMap()) {
      *error = "vehicle YAML is not a mapping";
      return std::nullopt;
    }
    model.name = Get<std::string>(root, "name", "");
    model.wheelbase_m = Get(root, "wheelbase", model.wheelbase_m);
    model.length_m = Get(root, "length", model.length_m);
    model.width_m = Get(root, "width", model.width_m);
    model.rear_axle_offset_x_m =
        Get(root, "rear_axle_offset_x", model.rear_axle_offset_x_m);
    model.max_steer_angle_rad =
        Get(root, "max_steer_angle", model.max_steer_angle_rad);
    model.tau_steer_s = Get(root, "tau_steer", model.tau_steer_s);
    model.tau_throttle_s = Get(root, "tau_throttle", model.tau_throttle_s);
    model.wheelbase_fitted_m = Get(root, "wheelbase_fitted", model.wheelbase_m);
    model.understeer_gradient = Get(root, "understeer_gradient", 0.0);
    const YAML::Node limits = root["limits"];
    if (limits.IsDefined() && limits.IsMap()) {
      model.limits.a_max_mps2 = Get(limits, "a_max", model.limits.a_max_mps2);
      model.limits.a_min_mps2 = Get(limits, "a_min", model.limits.a_min_mps2);
      model.limits.jerk_max_mps3 =
          Get(limits, "jerk_max", model.limits.jerk_max_mps3);
      model.limits.jerk_brake_max_mps3 =
          Get(limits, "jerk_brake_max", model.limits.jerk_max_mps3);
      model.limits.steer_rate_max_radps =
          Get(limits, "steer_rate_max", model.limits.steer_rate_max_radps);
      model.limits.kappa_max = Get(limits, "kappa_max", model.limits.kappa_max);
    }
  } catch (const YAML::Exception& e) {
    *error = std::string("vehicle YAML: ") + e.what();
    return std::nullopt;
  }
  model.longitudinal_map = LongitudinalMap::FromYamlString(text, error);
  if (!model.longitudinal_map.has_value()) {
    return std::nullopt;
  }
  return model;
}

// Slurps the file and delegates to ParseVehicleModel.
std::optional<VehicleModel> LoadVehicleModel(const std::string& path,
                                             std::string* error) {
  const std::ifstream in(path);
  if (!in) {
    *error = "cannot open " + path;
    return std::nullopt;
  }
  std::stringstream buffer;
  buffer << in.rdbuf();
  return ParseVehicleModel(buffer.str(), error);
}

}  // namespace nuway_control
