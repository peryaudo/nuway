cmake_minimum_required(VERSION 3.28)
project(nuway_control LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(nuway_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nuway_common REQUIRED)
find_package(nuway_msgs REQUIRED)

# Library: no rclcpp. Position independent because nuway_py links it into
# the pybind11 extension module.
add_library(
    nuway_control_lib
    src/longitudinal_map.cpp
    src/vehicle_model.cpp
    src/pure_pursuit_pid.cpp
)
target_include_directories(
    nuway_control_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
    nuway_control_lib
    PUBLIC Eigen3::Eigen nuway_common::nuway_common
    PRIVATE yaml-cpp::yaml-cpp
)
set_target_properties(nuway_control_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
nuway_target_defaults(nuway_control_lib)

add_executable(pure_pursuit_pid_node src/pure_pursuit_pid_node.cpp)
target_link_libraries(
    pure_pursuit_pid_node
    PRIVATE
        nuway_control_lib
        rclcpp::rclcpp
        ${nuway_msgs_TARGETS}
        nuway_common::nuway_common_ros
)
nuway_target_defaults(pure_pursuit_pid_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS nuway_control_lib EXPORT export_${PROJECT_NAME})
install(TARGETS pure_pursuit_pid_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    foreach(unit longitudinal_map pure_pursuit_pid)
        ament_add_gtest(${unit}_test test/${unit}_test.cpp)
        target_link_libraries(${unit}_test nuway_control_lib)
        target_compile_definitions(
            ${unit}_test
            PRIVATE NUWAY_REPO_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../../.."
        )
        nuway_target_defaults(${unit}_test)
    endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 yaml-cpp nuway_common)
ament_package()
