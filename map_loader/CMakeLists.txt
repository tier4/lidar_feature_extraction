cmake_minimum_required(VERSION 3.8)
project(lidar_feature_map_loader)

set(CMAKE_CXX_FLAGS "-g -Wall -Wextra -Wpedantic -Werror -fprofile-arcs -ftest-coverage")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_lib SHARED src/map_loader.cpp)
rclcpp_components_register_node(
  ${PROJECT_NAME}_lib
  PLUGIN "MapLoaderNode"
  EXECUTABLE ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
