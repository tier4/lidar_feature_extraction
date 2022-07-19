cmake_minimum_required(VERSION 3.14)
project(ekf_localizer)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-pg -g -Wall -Wextra -Wpedantic -Werror -fprofile-arcs -ftest-coverage")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_executable(ekf_localizer
  src/ekf_localizer_node.cpp
  src/ekf_localizer.cpp
  src/warning.cpp
)
target_include_directories(ekf_localizer PUBLIC include)

ament_target_dependencies(
  ekf_localizer
  fmt kalman_filter lidar_feature_library rotationlib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(ekf_localizer-test test/test_ekf_localizer.test
    test/src/test_ekf_localizer.cpp
    src/ekf_localizer.cpp
    src/warning.cpp
  )
  target_include_directories(ekf_localizer-test
    PRIVATE
    include
  )
  ament_target_dependencies(
    ekf_localizer-test
    fmt geometry_msgs kalman_filter lidar_feature_library
    nav_msgs rclcpp rotationlib tf2 tf2_ros)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
