cmake_minimum_required(VERSION 3.28)
project(nuway_prediction LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(nuway_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nuway_common REQUIRED)
find_package(nuway_map REQUIRED)
find_package(nuway_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

# Library: no rclcpp. Position independent because nuway_py binds it for
# the M6 expert.
add_library(nuway_prediction_lib src/const_vel.cc)
target_include_directories(
    nuway_prediction_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
    nuway_prediction_lib
    PUBLIC Eigen3::Eigen nuway_common::nuway_common nuway_map::nuway_map_lib
)
set_target_properties(
    nuway_prediction_lib
    PROPERTIES POSITION_INDEPENDENT_CODE ON
)
nuway_target_defaults(nuway_prediction_lib)

add_executable(const_vel_node src/const_vel_node.cc)
target_link_libraries(
    const_vel_node
    PRIVATE
        nuway_prediction_lib
        rclcpp::rclcpp
        ${nuway_msgs_TARGETS}
        ${rosgraph_msgs_TARGETS}
        nuway_common::nuway_common_ros
)
nuway_target_defaults(const_vel_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS nuway_prediction_lib EXPORT export_${PROJECT_NAME})
install(TARGETS const_vel_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(const_vel_test test/const_vel_test.cc)
    target_link_libraries(const_vel_test nuway_prediction_lib)
    nuway_target_defaults(const_vel_test)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 nuway_common nuway_map nuway_msgs)
ament_package()
