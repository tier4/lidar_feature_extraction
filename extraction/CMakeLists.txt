cmake_minimum_required(VERSION 3.16)
project(lidar_feature_extraction)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS
  "-pg -O3 -Wall -Wextra -Wpedantic -Werror -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)
set(BUILD_TESTING ON)

find_package(fmt REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

set(SOURCE_FILES
  src/convolution.cpp
  src/ring.cpp
  src/iterator.cpp
  src/index_range.cpp
  src/color_points.cpp
  src/math.cpp
  src/curvature.cpp)

ament_auto_add_library(lidar_feature_extraction_lib SHARED ${SOURCE_FILES})
target_link_libraries(lidar_feature_extraction_lib fmt)

ament_auto_add_executable(lidar_feature_extraction app/feature_extraction.cpp)
target_link_libraries(lidar_feature_extraction lidar_feature_extraction_lib)

function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})

  message("test_name = ${test_name}")

  ament_add_gmock(${test_name} ${filepath})
  target_include_directories(${test_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(${test_name} lidar_feature_extraction_lib)
  ament_target_dependencies(${test_name} ${lidar_feature_extraction_FOUND_BUILD_DEPENDS})
endfunction()

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )

  ament_lint_auto_find_test_dependencies()

  file(GLOB SOURCE_FILES src/* include/*)

  find_package(ament_cmake_cpplint)
  ament_cpplint(${SOURCE_FILES})

  find_package(ament_cmake_uncrustify)
  ament_uncrustify(${SOURCE_FILES})

  file(GLOB TEST_FILES test/*)
  foreach (filepath ${TEST_FILES})
    add_testcase(${filepath})
  endforeach()
endif()

ament_auto_package()
