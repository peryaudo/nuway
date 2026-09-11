cmake_minimum_required(VERSION 3.28)
project(nuway_viz LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(nuway_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nuway_common REQUIRED)
find_package(nuway_msgs REQUIRED)

add_executable(marker_node src/marker_node.cc)
target_include_directories(
    marker_node
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(
    marker_node
    PRIVATE
        rclcpp::rclcpp
        ${sensor_msgs_TARGETS}
        ${visualization_msgs_TARGETS}
        nuway_common::nuway_common_ros
        ${nuway_msgs_TARGETS}
)
nuway_target_defaults(marker_node)

install(TARGETS marker_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY foxglove DESTINATION share/${PROJECT_NAME})

ament_package()
