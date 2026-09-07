// Declaration of the nuway_control binding assembled by bindings.cpp.
// Introduced in M0.
#ifndef NUWAY_PY_SRC_BIND_CONTROL_HPP_
#define NUWAY_PY_SRC_BIND_CONTROL_HPP_

#include <pybind11/pybind11.h>

namespace nuway_py {

// Binds nuway_control: LongitudinalMap and its interpolation helpers.
void BindControl(pybind11::module_& module);

}  // namespace nuway_py

#endif  // NUWAY_PY_SRC_BIND_CONTROL_HPP_
