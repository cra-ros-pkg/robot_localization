cmake_minimum_required(VERSION 3.5)
project(robot_localization)

if(NOT WIN32)
  add_definitions(-std=c++11)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  #"srv/SetDatum.srv"
  "srv/SetPose.srv"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    #geographic_msgs
  ADD_LINTER_TESTS
)

include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})

add_library(
  filter_utilities
  src/filter_utilities.cpp)

add_library(
  filter_base
  src/filter_base.cpp)
target_link_libraries(
  filter_base
  filter_utilities)

add_library(
  ekf
  src/ekf.cpp)
target_link_libraries(
  ekf
  filter_base)

add_library(
  ukf
  src/ukf.cpp)
target_link_libraries(
  ukf
  filter_base)

install(
  TARGETS
  ekf
  filter_base
  filter_utilities
  ukf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install(
#  TARGETS ${PROJECT_NAME}
#  DESTINATION lib/${PROJECT_NAME}
#)

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  
  # This forces cppcheck to consider all files in this project to be C++,
  # including the headers which end with .h, which cppcheck would normally
  # consider to be C instead.
  ament_cppcheck(LANGUAGE "c++")
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(
  ekf
  filter_base
  filter_utilities
  ukf
)

ament_package()
