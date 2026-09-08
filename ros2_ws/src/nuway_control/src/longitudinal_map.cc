// LongitudinalMap: bilinear table lookups and their monotone inverse. The
// algorithms are described in the header; this file carries the numerics
// (bin search, clamping, the numpy conventions the Python twin follows).
#include "nuway_control/longitudinal_map.h"

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <sstream>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace nuway_control {

// Blends the two table rows bracketing v: row = row_i + alpha (row_{i+1} -
// row_i) with alpha in [0, 1] the position of v inside [v_bins[i],
// v_bins[i+1]]. v outside the bins is clamped to the edge row (alpha 0 or 1),
// so the map saturates in speed rather than extrapolating a fit that was
// never measured there.
std::vector<double> InterpRow(const std::vector<double>& v_bins,
                              const Table& table, double v) {
  const double v_clamped = std::clamp(v, v_bins.front(), v_bins.back());
  // searchsorted(side="right") - 1, clamped to the last segment start.
  const auto upper = std::upper_bound(v_bins.begin(), v_bins.end(), v_clamped);
  std::ptrdiff_t idx = std::distance(v_bins.begin(), upper) - 1;
  idx = std::max<std::ptrdiff_t>(
      0, std::min<std::ptrdiff_t>(
             idx, static_cast<std::ptrdiff_t>(v_bins.size()) - 2));
  const auto i = static_cast<std::size_t>(idx);
  const double span = v_bins[i + 1] - v_bins[i];
  const double alpha = span > 0.0 ? (v_clamped - v_bins[i]) / span : 0.0;
  std::vector<double> row(table[i].size());
  for (std::size_t j = 0; j < row.size(); ++j) {
    row[j] = table[i][j] + (alpha * (table[i + 1][j] - table[i][j]));
  }
  return row;
}

// Piecewise-linear evaluation of values(x) with the segment found by binary
// search; x is clamped to [bins.front(), bins.back()] first. The two early
// returns are the boundary cases of the upper_bound search (x below the
// first bin, x exactly on the last bin).
double Interp1d(const std::vector<double>& bins,
                const std::vector<double>& values, double x) {
  const double x_clamped = std::clamp(x, bins.front(), bins.back());
  // np.interp: bins[i] <= x < bins[i+1]; x == bins.back() returns the last.
  const auto upper = std::upper_bound(bins.begin(), bins.end(), x_clamped);
  const std::ptrdiff_t idx = std::distance(bins.begin(), upper) - 1;
  if (idx < 0) {
    return values.front();
  }
  const auto i = static_cast<std::size_t>(idx);
  if (i + 1 >= bins.size()) {
    return values.back();
  }
  const double span = bins[i + 1] - bins[i];
  const double alpha = span > 0.0 ? (x_clamped - bins[i]) / span : 0.0;
  return values[i] + (alpha * (values[i + 1] - values[i]));
}

// Solves values(x) = target for x on a monotone piecewise-linear curve. The
// direction is read off the end points; a target outside [min, max] of the
// curve saturates to the bin at the matching end (full pedal or no pedal).
// Otherwise the first segment that brackets the target is inverted linearly:
// x = bins[i] + (target - a) / (b - a) * (bins[i+1] - bins[i]). A flat
// segment (b == a) that contains the target returns its left end, the
// smallest pedal that reaches the target. Linear scan: the rows have ~10
// bins, so a binary search would not pay for itself.
double InvertMonotone(const std::vector<double>& bins,
                      const std::vector<double>& values, double target) {
  const bool increasing = values.back() >= values.front();
  const double lo = increasing ? values.front() : values.back();
  const double hi = increasing ? values.back() : values.front();
  if (target <= lo) {
    return increasing ? bins.front() : bins.back();
  }
  if (target >= hi) {
    return increasing ? bins.back() : bins.front();
  }
  for (std::size_t i = 0; i + 1 < bins.size(); ++i) {
    const double a = values[i];
    const double b = values[i + 1];
    if ((a <= target && target <= b) || (b <= target && target <= a)) {
      if (b == a) {
        return bins[i];
      }
      return bins[i] + ((target - a) / (b - a) * (bins[i + 1] - bins[i]));
    }
  }
  return bins.back();
}

namespace {

// Validates a bin vector: at least two entries, strictly increasing (the
// interpolation divides by the bin spacing and upper_bound assumes order).
bool CheckBins(const std::vector<double>& bins, const char* name,
               std::string* error) {
  if (bins.size() < 2) {
    *error =
        std::string(name) + " must be increasing with at least two entries";
    return false;
  }
  for (std::size_t i = 0; i + 1 < bins.size(); ++i) {
    if (bins[i + 1] <= bins[i]) {
      *error =
          std::string(name) + " must be increasing with at least two entries";
      return false;
    }
  }
  return true;
}

// Validates that `table` is exactly rows x cols (one row per v bin, one
// column per pedal bin); the lookups index it without bounds checks.
bool CheckTable(const Table& table, std::size_t rows, std::size_t cols,
                const char* name, std::string* error) {
  bool ok = table.size() == rows;
  for (const std::vector<double>& row : table) {
    ok = ok && row.size() == cols;
  }
  if (!ok) {
    std::ostringstream out;
    out << name << " shape != (" << rows << ", " << cols << ")";
    *error = out.str();
  }
  return ok;
}

}  // namespace

// Shape checks first (so a size mismatch is reported as such even when the
// bins are also bad), then bin ordering; only a fully consistent set of
// tables becomes a map.
std::optional<LongitudinalMap> LongitudinalMap::FromTables(
    std::vector<double> v_bins, std::vector<double> throttle_bins,
    Table accel_table, std::vector<double> brake_bins, Table decel_table,
    std::vector<double> coast_accel, std::string* error) {
  if (!CheckTable(accel_table, v_bins.size(), throttle_bins.size(),
                  "accel_table", error) ||
      !CheckTable(decel_table, v_bins.size(), brake_bins.size(), "decel_table",
                  error)) {
    return std::nullopt;
  }
  if (coast_accel.size() != v_bins.size()) {
    std::ostringstream out;
    out << "coast_accel shape (" << coast_accel.size() << ",) != ("
        << v_bins.size() << ",)";
    *error = out.str();
    return std::nullopt;
  }
  if (!CheckBins(v_bins, "v_bins", error) ||
      !CheckBins(throttle_bins, "throttle_bins", error) ||
      !CheckBins(brake_bins, "brake_bins", error)) {
    return std::nullopt;
  }
  LongitudinalMap map;
  map.v_bins_ = std::move(v_bins);
  map.throttle_bins_ = std::move(throttle_bins);
  map.accel_table_ = std::move(accel_table);
  map.brake_bins_ = std::move(brake_bins);
  map.decel_table_ = std::move(decel_table);
  map.coast_accel_ = std::move(coast_accel);
  return map;
}

// Reads the `longitudinal_map` block (any other top-level keys of the
// vehicle YAML are ignored here; vehicle_model.cc reads those).
std::optional<LongitudinalMap> LongitudinalMap::FromYamlString(
    const std::string& text, std::string* error) {
  // yaml-cpp reports malformed documents by throwing; this is the one place
  // in nuway_control that meets it, so the throw is turned into `error`.
  try {
    const YAML::Node root = YAML::Load(text);
    const YAML::Node cfg = root["longitudinal_map"];
    if (!cfg.IsDefined() || !cfg.IsMap()) {
      *error = "missing longitudinal_map block";
      return std::nullopt;
    }
    return FromTables(cfg["v_bins"].as<std::vector<double>>(),
                      cfg["throttle_bins"].as<std::vector<double>>(),
                      cfg["accel_table"].as<Table>(),
                      cfg["brake_bins"].as<std::vector<double>>(),
                      cfg["decel_table"].as<Table>(),
                      cfg["coast_accel"].as<std::vector<double>>(), error);
  } catch (const YAML::Exception& e) {
    *error = std::string("longitudinal_map: ") + e.what();
    return std::nullopt;
  }
}

// Slurps the file and delegates to FromYamlString.
std::optional<LongitudinalMap> LongitudinalMap::FromYamlFile(
    const std::string& path, std::string* error) {
  const std::ifstream in(path);
  if (!in) {
    *error = "cannot open " + path;
    return std::nullopt;
  }
  std::stringstream buffer;
  buffer << in.rdbuf();
  return FromYamlString(buffer.str(), error);
}

// Bilinear: blend the v rows, then interpolate along the throttle axis.
double LongitudinalMap::Accel(double v, double throttle) const {
  return Interp1d(throttle_bins_, InterpRow(v_bins_, accel_table_, v),
                  throttle);
}

// Bilinear: blend the v rows, then interpolate along the brake axis.
double LongitudinalMap::Decel(double v, double brake) const {
  return Interp1d(brake_bins_, InterpRow(v_bins_, decel_table_, v), brake);
}

// 1-D over the speed bins; this is also the throttle/brake split point.
double LongitudinalMap::Coast(double v) const {
  return Interp1d(v_bins_, coast_accel_, v);
}

// Throttle when the request is reachable without braking (accel_des >=
// Coast(v): throttle 0 already gives the coast row's first entry), brake
// otherwise. The inverted row is the same blended row the forward lookup
// uses, so Accel(v, Inverse(v, a).throttle) == a wherever a is inside the
// row's range.
PedalCommand LongitudinalMap::Inverse(double v, double accel_des) const {
  if (accel_des >= Coast(v)) {
    return PedalCommand{
        InvertMonotone(throttle_bins_, InterpRow(v_bins_, accel_table_, v),
                       accel_des),
        0.0};
  }
  return PedalCommand{
      0.0, InvertMonotone(brake_bins_, InterpRow(v_bins_, decel_table_, v),
                          accel_des)};
}

}  // namespace nuway_control
