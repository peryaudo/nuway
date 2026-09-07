cmake_minimum_required(VERSION 3.28)
project(nuway_route LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(nuway_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nuway_common REQUIRED)
find_package(nuway_map REQUIRED)
find_package(nuway_msgs REQUIRED)

# Library: no rclcpp.
add_library(
    nuway_route_lib
    src/route_planner.cpp
    src/reference_line_builder.cpp
)
target_include_directories(
    nuway_route_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
    nuway_route_lib
    PUBLIC
        Eigen3::Eigen
        nuway_common::nuway_common
        nuway_map::nuway_map_lib
        ${nuway_msgs_TARGETS}
)
nuway_target_defaults(nuway_route_lib)

add_executable(route_planner_node src/route_planner_node.cpp)
target_link_libraries(
    route_planner_node
    PRIVATE
        nuway_route_lib
        rclcpp::rclcpp
        ${nav_msgs_TARGETS}
        nuway_common::nuway_common_ros
)
nuway_target_defaults(route_planner_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS nuway_route_lib EXPORT export_${PROJECT_NAME})
install(TARGETS route_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    foreach(unit route_planner reference_line_builder)
        ament_add_gtest(${unit}_test test/${unit}_test.cpp)
        target_link_libraries(${unit}_test nuway_route_lib)
        target_compile_definitions(
            ${unit}_test
            PRIVATE NUWAY_REPO_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../../.."
        )
        nuway_target_defaults(${unit}_test)
    endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 nuway_common nuway_map nuway_msgs)
ament_package()
