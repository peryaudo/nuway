// pybind11 bindings of nuway_control (M0): LongitudinalMap for the parity
// test against nuway_ml/common/longitudinal_map.py. A load failure returns
// None (the library reports errors by value, never by throwing).
#include "bind_control.hpp"

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <pybind11/stl.h>

#include <nuway_control/longitudinal_map.hpp>

namespace nuway_py {

namespace py = pybind11;

void BindControl(py::module_& module) {
  using nuway_control::LongitudinalMap;
  using nuway_control::PedalCommand;
  using nuway_control::Table;
  module.def("interp_row", &nuway_control::InterpRow, py::arg("v_bins"),
             py::arg("table"), py::arg("v"));
  module.def("interp_1d", &nuway_control::Interp1d, py::arg("bins"),
             py::arg("values"), py::arg("x"));
  module.def("invert_monotone", &nuway_control::InvertMonotone, py::arg("bins"),
             py::arg("values"), py::arg("target"));
  py::class_<LongitudinalMap>(module, "LongitudinalMap")
      .def_static(
          "from_tables",
          [](std::vector<double> v_bins, std::vector<double> throttle_bins,
             Table accel_table, std::vector<double> brake_bins,
             Table decel_table, std::vector<double> coast_accel) {
            std::string error;
            return LongitudinalMap::FromTables(
                std::move(v_bins), std::move(throttle_bins),
                std::move(accel_table), std::move(brake_bins),
                std::move(decel_table), std::move(coast_accel), &error);
          },
          py::arg("v_bins"), py::arg("throttle_bins"), py::arg("accel_table"),
          py::arg("brake_bins"), py::arg("decel_table"), py::arg("coast_accel"))
      .def_static(
          "from_yaml",
          [](const std::string& path) {
            std::string error;
            return LongitudinalMap::FromYamlFile(path, &error);
          },
          py::arg("path"))
      .def_static(
          "from_yaml_string",
          [](const std::string& text) {
            std::string error;
            return LongitudinalMap::FromYamlString(text, &error);
          },
          py::arg("text"))
      .def_property_readonly("v_bins", &LongitudinalMap::v_bins)
      .def_property_readonly("throttle_bins", &LongitudinalMap::throttle_bins)
      .def_property_readonly("accel_table", &LongitudinalMap::accel_table)
      .def_property_readonly("brake_bins", &LongitudinalMap::brake_bins)
      .def_property_readonly("decel_table", &LongitudinalMap::decel_table)
      .def_property_readonly("coast_accel", &LongitudinalMap::coast_accel)
      .def("accel", &LongitudinalMap::Accel, py::arg("v"), py::arg("throttle"))
      .def("decel", &LongitudinalMap::Decel, py::arg("v"), py::arg("brake"))
      .def("coast", &LongitudinalMap::Coast, py::arg("v"))
      .def(
          "inverse",
          [](const LongitudinalMap& map, double v, double accel_des) {
            const PedalCommand cmd = map.Inverse(v, accel_des);
            return std::make_pair(cmd.throttle, cmd.brake);
          },
          py::arg("v"), py::arg("accel_des"));
}

}  // namespace nuway_py
