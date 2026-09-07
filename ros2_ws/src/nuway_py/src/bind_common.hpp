// Declarations of the per-library binding functions that bindings.cpp
// assembles into the nuway_py._core module. Introduced in M0.
#ifndef NUWAY_PY_SRC_BIND_COMMON_HPP_
#define NUWAY_PY_SRC_BIND_COMMON_HPP_

#include <pybind11/pybind11.h>

namespace nuway_py {

// Binds nuway_common: geometry, carla_conv, frenet, occupancy, tick.
void BindCommon(pybind11::module_& module);

}  // namespace nuway_py

#endif  // NUWAY_PY_SRC_BIND_COMMON_HPP_
