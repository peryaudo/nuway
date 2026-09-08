cmake_minimum_required(VERSION 3.28)
project(nuway_common LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(nuway_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nuway_msgs REQUIRED)

# Pure library: geometry, frenet, trajectory, occupancy, tick, carla_conv,
# frames. No rclcpp, so gtest targets link only this.
add_library(nuway_common INTERFACE)
target_include_directories(
    nuway_common
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
    nuway_common
    INTERFACE Eigen3::Eigen ${builtin_interfaces_TARGETS}
)
target_compile_features(nuway_common INTERFACE cxx_std_17)

# ROS-facing helpers: qos, diag, params. Nodes link this.
add_library(nuway_common_ros INTERFACE)
target_link_libraries(
    nuway_common_ros
    INTERFACE nuway_common rclcpp::rclcpp ${nuway_msgs_TARGETS}
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS nuway_common nuway_common_ros EXPORT export_${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    foreach(
        unit
        geometry
        carla_conv
        frenet
        trajectory
        occupancy
        tick
    )
        ament_add_gtest(${unit}_test test/${unit}_test.cc)
        target_link_libraries(${unit}_test nuway_common)
        nuway_target_defaults(${unit}_test)
    endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 builtin_interfaces rclcpp nuway_msgs)
ament_package()
