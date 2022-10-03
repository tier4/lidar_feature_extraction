cmake_minimum_required(VERSION 3.16)
project(lidar_feature_localization)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS
  "-pg -g -O3 -Wall -Wextra -Wpedantic -Werror -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(fmt REQUIRED)

ament_auto_find_build_dependencies()

set(INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/nanoflann/include/)

set(SOURCE_FILES
  src/alignment.cpp
  src/degenerate.cpp
  src/edge.cpp
  src/robust.cpp
  src/jacobian.cpp
  src/kdtree.cpp
  src/optimizer.cpp
  src/posevec.cpp)

ament_auto_add_library(lidar_feature_localization_lib SHARED ${SOURCE_FILES})
target_include_directories(lidar_feature_localization_lib PUBLIC ${INCLUDE_DIRS})

foreach(suffix IN ITEMS "localization" "odometry")
  ament_auto_add_executable("lidar_feature_${suffix}" app/${suffix}.cpp)
  target_link_libraries("lidar_feature_${suffix}" fmt lidar_feature_localization_lib)
endforeach()

function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})

  message("test_name = ${test_name}")

  ament_add_gmock(${test_name} ${filepath})
  target_link_libraries("${test_name}" fmt lidar_feature_localization_lib)
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()

if(BUILD_TESTING)
  file(GLOB
    FILES_TO_CHECK
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*
    ${CMAKE_CURRENT_SOURCE_DIR}/test/*)

  find_package(ament_cmake_cpplint)
  ament_cpplint(${FILES_TO_CHECK})

  find_package(ament_cmake_uncrustify)
  ament_uncrustify(${FILES_TO_CHECK})

  find_package(ament_cmake_gmock REQUIRED)

  set(TEST_FILES
    test/test_posevec.cpp
    test/test_optimizer.cpp
    test/test_jacobian.cpp
    test/test_edge.cpp
    test/test_recent_scans.cpp
    test/test_robust.cpp
    test/test_kdtree.cpp
    test/test_degenerate.cpp
    test/test_odometry.cpp
    test/test_pointcloud_to_matrix.cpp
    test/test_stamp_sorted_objects.cpp
    test/test_filter.cpp
    test/test_math.cpp
    test/test_alignment.cpp)

  foreach(filepath ${TEST_FILES})
    add_testcase(${filepath})
  endforeach()
endif()

ament_auto_package()
