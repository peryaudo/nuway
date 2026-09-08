// Entry point of the nuway_py._core extension module (M0). Each nuway C++
// library gets one Bind*() function in its own translation unit; this file
// only assembles them.
#include <pybind11/pybind11.h>

#include "bind_common.h"
#include "bind_control.h"

PYBIND11_MODULE(_core, module) {
  module.doc() = "nuway C++ libraries bound for parity tests and the M6 expert";
  nuway_py::BindCommon(module);
  nuway_py::BindControl(module);
}
