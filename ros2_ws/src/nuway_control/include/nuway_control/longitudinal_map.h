// Longitudinal actuator map: (speed, accel) <-> throttle / brake (M0 §2.6).
//
// CARLA's pedals are not accelerations: the same throttle yields a different
// acceleration at every speed (engine torque curve, gearing, drag), and a
// brake pedal's deceleration depends on speed too. tools/sysid measures the
// real behaviour with pedal sweeps and fits it as two 2-D lookup tables,
// a = f(v, throttle) and a = g(v, brake), plus the coast drag a_coast(v)
// with both pedals released (M0 §2.6; residual RMS in the YAML's sysid
// block). The tables come from configs/vehicle/<vehicle>.yaml:
//
//   longitudinal_map:
//     v_bins: [...]            # m/s, increasing
//     throttle_bins: [...]     # 0..1, increasing
//     accel_table: [[...]]     # a[v][throttle], m/s^2
//     brake_bins: [...]        # 0..1, increasing
//     decel_table: [[...]]     # a[v][brake], m/s^2 (<= 0)
//     coast_accel: [...]       # a[v] with no pedal
//
// Forward lookups are bilinear: the two table rows bracketing the speed are
// blended into one row (InterpRow), which is then interpolated over the
// pedal bins (Interp1d). The inverse pedal = h(v, a_des) is what the stack
// actually needs (control_adapter turns the controller's accel into pedals
// with it): it uses the same blended row, which is monotone in the pedal
// (more throttle never accelerates less, more brake never decelerates
// less), so one piecewise-linear root (InvertMonotone) recovers the pedal.
// Throttle versus brake is decided by comparing a_des with the coast
// acceleration: at or above coast the request is reachable with throttle
// alone (throttle 0 is coasting), below it only the brake gets there. Every
// lookup clamps to the table edges, so out-of-range speeds and requests
// saturate instead of extrapolating.
//
// Mirrored by nuway_ml/common/longitudinal_map.py (parity-tested through
// nuway_py); the free functions below are the twins of its module helpers.
#ifndef NUWAY_CONTROL_LONGITUDINAL_MAP_H_
#define NUWAY_CONTROL_LONGITUDINAL_MAP_H_

#include <optional>
#include <string>
#include <vector>

namespace nuway_control {

// A 2-D table indexed [row][col]; here always [v bin][pedal bin].
using Table = std::vector<std::vector<double>>;

// Row of `table` at speed v, linearly interpolated between v bins (clamped).
// The first half of the bilinear lookup: the result is a full pedal row.
std::vector<double> InterpRow(const std::vector<double>& v_bins,
                              const Table& table, double v);

// Linear interpolation of `values` over `bins` at x (clamped). Matches
// numpy.interp, including x == bins.back() returning values.back().
double Interp1d(const std::vector<double>& bins,
                const std::vector<double>& values, double x);

// The bin at which the piecewise-linear `values` reaches `target`. `values`
// must be monotone (either direction); saturates at the ends. This is the
// 1-D root finding of M0 §2.6 (u = h(v, a_des) per v row).
double InvertMonotone(const std::vector<double>& bins,
                      const std::vector<double>& values, double target);

// Pedal pair produced by LongitudinalMap::Inverse; at most one is non-zero.
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
  // orderings are inconsistent (accel_table must be |v_bins| x
  // |throttle_bins|, decel_table |v_bins| x |brake_bins|, coast_accel
  // |v_bins|, every bin vector strictly increasing with >= 2 entries).
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

  // Raw tables and bins, as loaded (for the pybind parity tests and tools).
  const std::vector<double>& v_bins() const { return v_bins_; }
  const std::vector<double>& throttle_bins() const { return throttle_bins_; }
  const Table& accel_table() const { return accel_table_; }
  const std::vector<double>& brake_bins() const { return brake_bins_; }
  const Table& decel_table() const { return decel_table_; }
  const std::vector<double>& coast_accel() const { return coast_accel_; }

  // Acceleration (m/s^2) at speed v with `throttle` applied (bilinear).
  double Accel(double v, double throttle) const;
  // Acceleration (m/s^2, <= 0) at speed v with `brake` applied (bilinear).
  double Decel(double v, double brake) const;
  // Acceleration with no pedal at speed v (drag, m/s^2).
  double Coast(double v) const;
  // Pedal command in [0, 1] that yields `accel_des` at v: throttle when
  // accel_des is at or above the coast acceleration, brake below it; each is
  // found by inverting the interpolated monotone table row and saturates
  // (a request beyond Accel(v, 1) or Decel(v, 1) gives the full pedal).
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
