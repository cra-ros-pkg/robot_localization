cmake_minimum_required(VERSION 3.5)
project(robot_localization)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

set(library_name rl_lib)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetDatum.srv"
  "srv/SetPose.srv"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    geographic_msgs
    diagnostic_msgs
  ADD_LINTER_TESTS
)

include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(
  ${library_name}
  src/ekf.cpp
  src/ukf.cpp
  src/filter_base.cpp
  src/filter_utilities.cpp
  src/navsat_transform.cpp
  src/ros_filter.cpp
  src/ros_filter_utilities.cpp)

rosidl_target_interfaces(${library_name}
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(
  ekf_node
  src/ekf_node.cpp
)

add_executable(
  ukf_node
  src/ukf_node.cpp
)

add_executable(
  navsat_transform_node
  src/navsat_transform_node.cpp
)

target_link_libraries(
  ${library_name}
  ${EIGEN3_LIBRARIES}
)

ament_target_dependencies(
  ${library_name}
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
  rcl
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_link_libraries(
  ekf_node
  ${library_name}
)

ament_target_dependencies(
  ekf_node
  rclcpp
)

target_link_libraries(
  ukf_node
  ${library_name}
)

ament_target_dependencies(
  ukf_node
  rclcpp
)

target_link_libraries(
  navsat_transform_node
  ${library_name}
)

ament_target_dependencies(
  navsat_transform_node
  rclcpp
)

install(
  TARGETS
  navsat_transform_node
  ekf_node
  ukf_node
  ${library_name}
  DESTINATION lib/${PROJECT_NAME}
)

# Install params config files.
install(DIRECTORY
  params
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_package()
