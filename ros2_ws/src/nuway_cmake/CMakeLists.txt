cmake_minimum_required(VERSION 3.28)
project(nuway_cmake LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS cmake/nuway_cmake-extras.cmake)
