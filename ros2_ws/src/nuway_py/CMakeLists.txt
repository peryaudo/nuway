cmake_minimum_required(VERSION 3.28)
project(nuway_py LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nuway_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(nuway_common REQUIRED)
find_package(nuway_control REQUIRED)

pybind11_add_module(
    _core
    src/bindings.cc
    src/bind_common.cc
    src/bind_control.cc
)
target_link_libraries(
    _core
    PRIVATE nuway_common::nuway_common nuway_control::nuway_control_lib
)
nuway_target_defaults(_core PYBIND)

ament_python_install_package(${PROJECT_NAME})
install(TARGETS _core DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")

ament_package()
