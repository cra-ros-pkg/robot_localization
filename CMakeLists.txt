cmake_minimum_required(VERSION 3.5)
project(robot_localization)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
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

set(libraries
  ${library_name} 
  ${angles_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${geographic_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${diagnostic_msgs_LIBRARIES}
  ${diagnostic_updater_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
)

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
  ${diagnostic_updater_INCLUDE_DIRS}
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

ament_target_dependencies(${library_name} 
  ${dependencies}
)

rosidl_target_interfaces(${library_name}
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(
  se_node
  src/se_node.cpp
)

add_executable(
  navsat_transform_node
  src/navsat_transform_node.cpp
)

target_link_libraries(
  ${libraries}
)

target_link_libraries(
  se_node
  ${library_name}
)

target_link_libraries(
  navsat_transform_node
  ${library_name}
)

ament_target_dependencies(
  se_node
  ${library_name}
  ${dependencies}
)

install(
  TARGETS
  navsat_transform_node
  se_node
  ${library_name}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)


  #### FILTER BASE TESTS ####
  ament_add_gtest(filter_base-test test/test_filter_base.cpp)
  target_link_libraries(filter_base-test ${library_name})

  # This test uses se_node node for convenience.
  ament_add_gtest(test_filter_base_diagnostics_timestamps
                    test/test_filter_base_diagnostics_timestamps.cpp)
  target_link_libraries(test_filter_base_diagnostics_timestamps ${library_name})
  add_dependencies(test_filter_base_diagnostics_timestamps se_node)

  #### EKF TESTS ######
  ament_add_gtest(test_ekf test/test_ekf.cpp)
  target_link_libraries(test_ekf ${library_name})

  ament_add_gtest(test_ekf_localization_node_interfaces
  test/test_ekf_localization_node_interfaces.cpp)
  target_link_libraries(test_ekf_localization_node_interfaces ${library_name})

  ament_add_gtest(test_ekf_localization_node_bag1 test/test_localization_node_bag_pose_tester.cpp)
  target_link_libraries(test_ekf_localization_node_bag1 ${library_name})

  ament_add_gtest(test_ekf_localization_node_bag2 test/test_localization_node_bag_pose_tester.cpp)
  target_link_libraries(test_ekf_localization_node_bag2 ${library_name})

  ament_add_gtest(test_ekf_localization_node_bag3 test/test_localization_node_bag_pose_tester.cpp)
  target_link_libraries(test_ekf_localization_node_bag3 ${library_name})

  #### UKF TESTS #####
  ament_add_gtest(test_ukf test/test_ukf.cpp)
  target_link_libraries(test_ukf ${library_name})

  ament_add_gtest(test_ukf_localization_node_interfaces test/test_ukf_localization_node_interfaces.cpp)
  target_link_libraries(test_ukf_localization_node_interfaces ${library_name})

  ament_add_gtest(test_ukf_localization_node_bag1 test/test_localization_node_bag_pose_tester.cpp)
  target_link_libraries(test_ukf_localization_node_bag1 ${library_name})

  ament_add_gtest(test_ukf_localization_node_bag2 test/test_localization_node_bag_pose_tester.cpp)
  target_link_libraries(test_ukf_localization_node_bag2 ${library_name})

  ament_add_gtest(test_ukf_localization_node_bag3 test/test_localization_node_bag_pose_tester.cpp)
  target_link_libraries(test_ukf_localization_node_bag3 ${library_name})

  # This forces cppcheck to consider all files in this project to be C++,
  # including the headers which end with .h, which cppcheck would normally
  # consider to be C instead.
  ament_cppcheck(LANGUAGE "c++")
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()

  install(
    TARGETS
    filter_base-test
    test_filter_base_diagnostics_timestamps
    test_ekf
    test_ekf_localization_node_interfaces
    test_ekf_localization_node_bag1
    test_ekf_localization_node_bag2
    test_ekf_localization_node_bag3
    test_ukf
    test_ukf_localization_node_interfaces
    test_ukf_localization_node_bag1
    test_ukf_localization_node_bag2
    test_ukf_localization_node_bag3
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install params config files.
install(DIRECTORY
  params
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
