cmake_minimum_required(VERSION 3.14)
project(ekf_localizer)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-g -O3 -Wall -Wextra -Wpedantic -Werror -fprofile-arcs -ftest-coverage")

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(ekf_localizer_lib SHARED
  src/check.cpp
  src/covariance.cpp
  src/ekf_localizer.cpp
  src/mahalanobis.cpp
  src/pose_measurement.cpp
  src/state_transition.cpp
  src/string.cpp
  src/tf.cpp
  src/twist_measurement.cpp
  src/update_interval.cpp
)

target_link_libraries(ekf_localizer_lib Eigen3::Eigen)

ament_auto_add_executable(ekf_localizer src/ekf_localizer_node.cpp)
target_link_libraries(ekf_localizer ekf_localizer_lib)
target_include_directories(ekf_localizer PUBLIC include)

function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})

  ament_add_gtest(${test_name} ${filepath})
  target_link_libraries("${test_name}" ekf_localizer_lib)
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

  set(TEST_FILES
    test/test_check.cpp
    # test/test_ekf_localizer.cpp
    test/test_mahalanobis.cpp
    test/test_measurement.cpp
    test/test_pose_measurement.cpp
    test/test_state_transition.cpp
    test/test_string.cpp
    test/test_tf.cpp
    test/test_update_interval.cpp)

  foreach(filepath ${TEST_FILES})
    add_testcase(${filepath})
  endforeach()
endif()

ament_auto_package()
