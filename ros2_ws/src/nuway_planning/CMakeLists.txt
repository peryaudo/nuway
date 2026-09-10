cmake_minimum_required(VERSION 3.28)
project(nuway_planning LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(nuway_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nuway_common REQUIRED)
find_package(nuway_control REQUIRED)
find_package(nuway_map REQUIRED)
find_package(nuway_msgs REQUIRED)
find_package(osqp_eigen_vendor REQUIRED)

# Library: no rclcpp. Position independent because nuway_py binds it for
# the M6 expert and gt_planning_node.
add_library(
    nuway_planning_lib
    src/route_line.cc
    src/behavior_fsm.cc
    src/lattice_sampler.cc
    src/collision_checker.cc
    src/piecewise_jerk_qp.cc
    src/candidate_refiner.cc
    src/rule_selector.cc
    src/planner.cc
)
target_include_directories(
    nuway_planning_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
    nuway_planning_lib
    PUBLIC
        Eigen3::Eigen
        nuway_common::nuway_common
        nuway_control::nuway_control_lib
        nuway_map::nuway_map_lib
    PRIVATE OsqpEigen::OsqpEigen
)
set_target_properties(
    nuway_planning_lib
    PROPERTIES POSITION_INDEPENDENT_CODE ON
)
nuway_target_defaults(nuway_planning_lib)

foreach(node behavior_fsm_node planner_node)
    add_executable(${node} src/${node}.cc)
    target_link_libraries(
        ${node}
        PRIVATE
            nuway_planning_lib
            rclcpp::rclcpp
            ${nuway_msgs_TARGETS}
            nuway_common::nuway_common_ros
    )
    nuway_target_defaults(${node})
    install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS nuway_planning_lib EXPORT export_${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    foreach(
        unit
        route_line
        behavior_fsm
        lattice_sampler
        collision_checker
        piecewise_jerk_qp
        candidate_refiner
        rule_selector
    )
        ament_add_gtest(${unit}_test test/${unit}_test.cc)
        target_link_libraries(${unit}_test nuway_planning_lib)
        nuway_target_defaults(${unit}_test)
    endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    Eigen3
    nuway_common
    nuway_control
    nuway_map
    nuway_msgs
    osqp_eigen_vendor
)
ament_package()
