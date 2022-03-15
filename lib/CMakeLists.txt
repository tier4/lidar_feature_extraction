cmake_minimum_required(VERSION 3.8)
project(lidar_feature_library)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-g -Wall -Wextra -Wpedantic -Werror -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)
set(BUILD_TESTING ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

find_package(fmt REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

add_library(${PROJECT_NAME} src/lib.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} fmt)

ament_target_dependencies(${PROJECT_NAME}
  "pcl_conversions"
  "pcl_ros"
  "sensor_msgs"
  "tf2_eigen"
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(PCL)

function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})

  message("test_name = ${test_name}")

  ament_add_gmock(${test_name} ${filepath})
  target_include_directories(${test_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(${test_name} fmt)
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
endfunction()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )

  ament_lint_auto_find_test_dependencies()

  file(GLOB SOURCE_FILES include/lidar_feature_library/*)

  find_package(ament_cmake_cpplint)
  ament_cpplint(${SOURCE_FILES})

  find_package(ament_cmake_uncrustify)
  ament_uncrustify(${SOURCE_FILES})

  file(GLOB TEST_FILES test/*)
  foreach(filepath ${TEST_FILES})
    add_testcase(${filepath})
  endforeach()
endif()

# ament_auto_package(INSTALL_TO_SHARE include)
install(DIRECTORY include/lidar_feature_library DESTINATION include)

install(
  TARGETS lidar_feature_library
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
