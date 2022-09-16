cmake_minimum_required(VERSION 3.14)
project(kalman_filter)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(kalman_filter SHARED
  src/kalman_filter.cpp
  src/time_delay_kalman_filter.cpp
  include/kalman_filter/kalman_filter.hpp
  include/kalman_filter/time_delay_kalman_filter.hpp
)

function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})

  ament_add_gtest(${test_name} ${filepath})
  target_link_libraries("${test_name}" kalman_filter)
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  file(GLOB
    FILES_TO_CHECK
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*
    ${CMAKE_CURRENT_SOURCE_DIR}/test/*)

  find_package(ament_cmake_cpplint)
  ament_cpplint(${FILES_TO_CHECK})

  find_package(ament_cmake_uncrustify)
  ament_uncrustify(${FILES_TO_CHECK})

  set(TEST_FILES test/test_matrix_size.cpp)

  foreach(filepath ${TEST_FILES})
    add_testcase(${filepath})
  endforeach()
endif()

ament_auto_package()
