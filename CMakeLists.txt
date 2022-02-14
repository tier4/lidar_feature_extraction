cmake_minimum_required(VERSION 3.10)
project(lidar_feature_extraction)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -pthread")

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME} DIRECTORY src)

find_package(fmt REQUIRED)
target_link_libraries(${PROJECT_NAME} fmt)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )

  ament_lint_auto_find_test_dependencies()

  set(TEST_FILES src/feature_extraction.cpp include/utility.hpp)

  find_package(ament_cmake_cpplint)
  ament_cpplint(${TEST_FILES})

  find_package(ament_cmake_uncrustify)
  ament_uncrustify(${TEST_FILES})

  ament_add_gtest(test_utility test/test_utility.cpp)
  target_include_directories(test_utility PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_utility fmt)
  ament_target_dependencies(test_utility ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
