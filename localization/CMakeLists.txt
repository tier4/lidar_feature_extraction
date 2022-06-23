cmake_minimum_required(VERSION 3.16)
project(lidar_feature_localization)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-pg -g -Wall -Wextra -Wpedantic -Werror -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

set(INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/nanoflann/include/)

file(GLOB HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/lidar_feature_localization/*)
list(APPEND HEADER_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/nanoflann/include/nanoflann.hpp)

ament_auto_add_executable("lidar_feature_localization" src/localization.cpp)
ament_auto_add_executable("lidar_feature_odometry" src/odometry.cpp)

target_include_directories("lidar_feature_localization" PUBLIC ${INCLUDE_DIRS})
target_include_directories("lidar_feature_odometry" PUBLIC ${INCLUDE_DIRS})

target_precompile_headers("lidar_feature_localization" PUBLIC ${HEADER_FILES})
target_precompile_headers("lidar_feature_odometry" PUBLIC ${HEADER_FILES})

function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})

  message("test_name = ${test_name}")

  ament_add_gmock(${test_name} ${filepath})
  target_include_directories(${test_name} PUBLIC ${INCLUDE_DIRS})
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

  file(GLOB TEST_FILES test/*)
  foreach(filepath ${TEST_FILES})
    add_testcase(${filepath})
  endforeach()
endif()

ament_auto_package()
