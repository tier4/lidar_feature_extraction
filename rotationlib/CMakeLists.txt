cmake_minimum_required(VERSION 3.16)
project(rotationlib)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS
  "-pg -O3 -Wall -Wextra -Wpedantic -Werror -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(BUILD_TESTING ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

set(SOURCE_FILES
  src/hat.cpp
  src/quaternion.cpp
  src/jacobian/quaternion.cpp)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED ${SOURCE_FILES})

function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})

  ament_add_gmock(${test_name} ${filepath})
  target_link_libraries(${test_name} ${PROJECT_NAME}_lib)
  target_include_directories(${test_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/testlib)
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()


if(BUILD_TESTING)
  set(TEST_FILES
    test/test_hat.cpp
    test/test_quaternion.cpp
    test/test_jacobian_quaternion.cpp)

  find_package(ament_lint_auto REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )

  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE SOURCE_AND_HEADER src/* include/*)

  find_package(ament_cmake_cpplint)
  ament_cpplint(${SOURCE_AND_HEADER})

  find_package(ament_cmake_uncrustify)
  ament_uncrustify(${SOURCE_AND_HEADER})

  foreach(filepath ${TEST_FILES})
    add_testcase(${filepath})
  endforeach()
endif()

ament_auto_package()
