cmake_minimum_required(VERSION 3.16)
project(tf_generator)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-g -Wall -Wextra -Wpedantic -Werror -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME} DIRECTORY src)

if(BUILD_TESTING)
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
endif()

ament_auto_package()
