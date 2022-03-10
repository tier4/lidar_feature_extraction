cmake_minimum_required(VERSION 3.8)
project(lidar_feature_localization)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-g -Wall -Wextra -Wpedantic -Werror -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

file(GLOB SRC_FILES src/*)
ament_auto_add_executable(${PROJECT_NAME} ${SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)


function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})

  message("test_name = ${test_name}")

  ament_add_gmock(${test_name} ${filepath})
  target_include_directories(${test_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()

set(BUILD_TESTING ON)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )

  ament_lint_auto_find_test_dependencies()

  file(GLOB SOURCE_AND_HEADER src/* include/*)

  find_package(ament_cmake_cpplint)
  ament_cpplint(${SOURCE_AND_HEADER})

  find_package(ament_cmake_uncrustify)
  ament_uncrustify(${SOURCE_AND_HEADER})

  file(GLOB TEST_FILES test/*)
  foreach(filepath ${TEST_FILES})
    add_testcase(${filepath})
  endforeach()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
