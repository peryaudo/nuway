cmake_minimum_required(VERSION 3.28)
project(osqp_eigen_vendor LANGUAGES CXX)

# The ROS vendor-package pattern (docs/03 §6.2): the upstream source is
# fetched at a release tag pinned by commit hash and built as an external
# project into this package's install prefix, so downstream packages get the
# OsqpEigen::OsqpEigen target through find_package(osqp_eigen_vendor). OSQP
# itself (0.6.2) comes from ros-jazzy-osqp-vendor; v0.8.1 is the last
# osqp-eigen release for the OSQP 0.6 API (v0.9.0 moved to OSQP 1.0).
find_package(ament_cmake REQUIRED)
find_package(osqp REQUIRED)
find_package(Eigen3 REQUIRED)

set(OSQP_EIGEN_TAG v0.8.1)
set(OSQP_EIGEN_COMMIT 85c37623774c682db396505f0d4ea677040c2557)

include(ExternalProject)
# CMAKE_PREFIX_PATH is a list; ExternalProject splits CMAKE_ARGS on ';', so
# the list is passed with a substitute separator.
string(REPLACE ";" "|" NUWAY_PREFIX_PATH "${CMAKE_PREFIX_PATH}")
ExternalProject_Add(
    osqp_eigen_external
    GIT_REPOSITORY https://github.com/robotology/osqp-eigen.git
    GIT_TAG ${OSQP_EIGEN_COMMIT}
    GIT_SHALLOW OFF
    GIT_PROGRESS OFF
    UPDATE_DISCONNECTED ON
    LIST_SEPARATOR |
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
        -DCMAKE_PREFIX_PATH=${NUWAY_PREFIX_PATH} -DCMAKE_BUILD_TYPE=Release
        -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF -DOSQP_EIGEN_DEBUG_OUTPUT=OFF
        -DENABLE_RPATH=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON
)

ament_package(CONFIG_EXTRAS cmake/osqp_eigen_vendor-extras.cmake)
