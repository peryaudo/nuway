cmake_minimum_required(VERSION 3.28)
project(nuway_map LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(nuway_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pugixml REQUIRED)
find_package(nanoflann REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nuway_common REQUIRED)
find_package(nuway_msgs REQUIRED)

# Library: no rclcpp.
add_library(nuway_map_lib src/opendrive_parser.cpp src/lane_graph.cpp)
target_include_directories(
    nuway_map_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
    nuway_map_lib
    PUBLIC
        Eigen3::Eigen
        nanoflann::nanoflann
        nuway_common::nuway_common
        ${nuway_msgs_TARGETS}
    PRIVATE pugixml::pugixml
)
nuway_target_defaults(nuway_map_lib)

add_executable(map_server_node src/map_server_node.cpp)
target_link_libraries(
    map_server_node
    PRIVATE nuway_map_lib rclcpp::rclcpp nuway_common::nuway_common_ros
)
nuway_target_defaults(map_server_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS nuway_map_lib EXPORT export_${PROJECT_NAME})
install(TARGETS map_server_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    foreach(unit opendrive_parser lane_graph)
        ament_add_gtest(${unit}_test test/${unit}_test.cpp)
        target_link_libraries(${unit}_test nuway_map_lib)
        target_compile_definitions(
            ${unit}_test
            PRIVATE NUWAY_REPO_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../../.."
        )
        nuway_target_defaults(${unit}_test)
    endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 nanoflann nuway_common nuway_msgs)
ament_package()
