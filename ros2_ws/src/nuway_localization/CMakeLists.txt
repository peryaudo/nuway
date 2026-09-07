cmake_minimum_required(VERSION 3.28)
project(nuway_localization LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(nuway_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nuway_common REQUIRED)
find_package(nuway_msgs REQUIRED)

add_executable(gt_pose_node src/gt_pose_node.cpp)
target_include_directories(
    gt_pose_node
    PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
    gt_pose_node
    PRIVATE
        Eigen3::Eigen
        rclcpp::rclcpp
        tf2_ros::tf2_ros
        ${geometry_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        nuway_common::nuway_common_ros
        ${nuway_msgs_TARGETS}
)
nuway_target_defaults(gt_pose_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS gt_pose_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
