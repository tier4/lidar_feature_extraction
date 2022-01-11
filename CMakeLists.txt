cmake_minimum_required(VERSION 3.10)
project(lio_sam)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -pthread")

find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(range-v3 REQUIRED)
find_package(fmt REQUIRED)

ament_auto_find_build_dependencies()
ament_auto_add_executable(${PROJECT_NAME}_imageProjection
  src/imageProjection.cpp
  include/utility.hpp
)

ament_target_dependencies(${PROJECT_NAME}_imageProjection
  geometry_msgs
  GTSAM
  nav_msgs
  pcl_conversions
  range-v3
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_auto_package(INSTALL_TO_SHARE launch)
