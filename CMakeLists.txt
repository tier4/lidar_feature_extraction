cmake_minimum_required(VERSION 3.10)
project(lidar_feature_extraction)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -pthread --coverage")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME} DIRECTORY src)

find_package(fmt REQUIRED)
target_link_libraries(${PROJECT_NAME} fmt)

function(add_testcase test_name)
  ament_add_gmock(${test_name} test/${test_name}.cpp)
  target_include_directories(${test_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(${test_name} fmt)
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )

  ament_lint_auto_find_test_dependencies()

  file(GLOB TEST_FILES src/* include/*)

  find_package(ament_cmake_cpplint)
  ament_cpplint(${TEST_FILES})

  find_package(ament_cmake_uncrustify)
  ament_uncrustify(${TEST_FILES})

  add_testcase(test_algorithm)
  add_testcase(test_extraction)
  add_testcase(test_index_range)
  add_testcase(test_mask)
  add_testcase(test_math)
  add_testcase(test_range)
  add_testcase(test_ring)
  add_testcase(test_utility)
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
