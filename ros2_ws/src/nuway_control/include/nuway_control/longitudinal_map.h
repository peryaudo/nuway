// Longitudinal actuator map: (speed, accel) <-> throttle / brake (M0 §2.6).
// The tables come from the sysid fit in configs/vehicle/<vehicle>.yaml:
//
//   longitudinal_map:
//     v_bins: [...]            # m/s, increasing
//     throttle_bins: [...]     # 0..1, increasing
//     accel_table: [[...]]     # a[v][throttle], m/s^2
//     brake_bins: [...]        # 0..1, increasing
//     decel_table: [[...]]     # a[v][brake], m/s^2 (<= 0)
//     coast_accel: [...]       # a[v] with no pedal
//
// Mirrored by nuway_ml/common/longitudinal_map.py (parity-tested through
// nuway_py); the free functions below are the twins of its module helpers.
#ifndef NUWAY_CONTROL_LONGITUDINAL_MAP_H_
#define NUWAY_CONTROL_LONGITUDINAL_MAP_H_

#include <optional>
#include <string>
#include <vector>

namespace nuway_control {

using Table = std::vector<std::vector<double>>;

// Row of `table` at speed v, linearly interpolated between v bins (clamped).
std::vector<double> InterpRow(const std::vector<double>& v_bins,
                              const Table& table, double v);

// Linear interpolation of `values` over `bins` at x (clamped).
double Interp1d(const std::vector<double>& bins,
                const std::vector<double>& values, double x);

// The bin at which the piecewise-linear `values` reaches `target`. `values`
// must be monotone (either direction); saturates at the ends.
double InvertMonotone(const std::vector<double>& bins,
                      const std::vector<double>& values, double target);

struct PedalCommand {
  double throttle = 0.0;  // 0..1
  double brake = 0.0;     // 0..1
};

// Forward tables and their inverse. Default-constructed maps are empty and
// only exist as value_or() fallbacks; use the From*() builders.
class LongitudinalMap {
 public:
  LongitudinalMap() = default;
  // Builds from the tables; nullopt with `error` set when shapes or bin
  // orderings are inconsistent.
  static std::optional<LongitudinalMap> FromTables(
      std::vector<double> v_bins, std::vector<double> throttle_bins,
      Table accel_table, std::vector<double> brake_bins, Table decel_table,
      std::vector<double> coast_accel, std::string* error);
  // Builds from the `longitudinal_map` block of a vehicle YAML document.
  static std::optional<LongitudinalMap> FromYamlString(const std::string& text,
                                                       std::string* error);
  // Builds from configs/vehicle/<vehicle>.yaml.
  static std::optional<LongitudinalMap> FromYamlFile(const std::string& path,
                                                     std::string* error);

  const std::vector<double>& v_bins() const { return v_bins_; }
  const std::vector<double>& throttle_bins() const { return throttle_bins_; }
  const Table& accel_table() const { return accel_table_; }
  const std::vector<double>& brake_bins() const { return brake_bins_; }
  const Table& decel_table() const { return decel_table_; }
  const std::vector<double>& coast_accel() const { return coast_accel_; }

  // Acceleration (m/s^2) at speed v with `throttle` applied.
  double Accel(double v, double throttle) const;
  // Acceleration (m/s^2, <= 0) at speed v with `brake` applied.
  double Decel(double v, double brake) const;
  // Acceleration with no pedal at speed v (drag, m/s^2).
  double Coast(double v) const;
  // Pedal command in [0, 1] that yields `accel_des` at v: throttle when
  // accel_des is at or above the coast acceleration, brake below it; each is
  // found by inverting the interpolated monotone table row and saturates.
  PedalCommand Inverse(double v, double accel_des) const;

 private:
  std::vector<double> v_bins_;
  std::vector<double> throttle_bins_;
  Table accel_table_;  // [v][throttle]
  std::vector<double> brake_bins_;
  Table decel_table_;                // [v][brake]
  std::vector<double> coast_accel_;  // [v]
};

}  // namespace nuway_control

#endif  // NUWAY_CONTROL_LONGITUDINAL_MAP_H_
