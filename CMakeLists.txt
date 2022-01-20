cmake_minimum_required(VERSION 3.10)
project(lidar_feature_extraction)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -pthread")

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}
  src/feature_extraction.cpp
  include/utility.hpp
)

ament_auto_package(INSTALL_TO_SHARE launch)
