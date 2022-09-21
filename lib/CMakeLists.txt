cmake_minimum_required(VERSION 3.16)
project(lidar_feature_library)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS
  "-pg -O3 -Wall -Wextra -Wpedantic -Werror -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

find_package(fmt REQUIRED)
find_package(PCL REQUIRED)

ament_auto_find_build_dependencies()

set(SOURCE_FILES
  src/algorithm.cpp
  src/lib.cpp
  src/eigen.cpp
  src/pcl_utils.cpp
  src/ros_msg.cpp
  src/random.cpp
  src/stats.cpp)

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

function(add_testcase filepath)
  find_package(ament_cmake_gmock REQUIRED)

  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})

  message("test_name = ${test_name}")

  ament_add_gmock(${test_name} ${filepath})
  target_link_libraries(${test_name} fmt ${PROJECT_NAME})
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )

  ament_lint_auto_find_test_dependencies()

  file(GLOB FILES_TO_CHECK include/lidar_feature_library/* src/*)

  find_package(ament_cmake_cpplint)
  ament_cpplint(${FILES_TO_CHECK})

  find_package(ament_cmake_uncrustify)
  ament_uncrustify(${FILES_TO_CHECK})

  file(GLOB TEST_FILES test/*)
  foreach(filepath ${TEST_FILES})
    add_testcase(${filepath})
  endforeach()
endif()

ament_auto_package()
