cmake_minimum_required(VERSION 2.8.3)
project(robot_localization)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  diagnostic_updater
  eigen_conversions
  geographic_msgs
  geometry_msgs
  message_filters
  message_generation
  nav_msgs
  roscpp
  roslint
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  xmlrpcpp)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP yaml-cpp)

# Attempt to find Eigen using its own CMake module.
# If that fails, fall back to cmake_modules package.
find_package(Eigen3)
set(EIGEN_PACKAGE EIGEN3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  set(EIGEN_PACKAGE Eigen)
endif()

add_definitions(-DEIGEN_NO_DEBUG -DEIGEN_MPL2_ONLY)

roslint_cpp()

###################################
## catkin specific configuration ##
###################################
add_service_files(
  FILES
    GetState.srv
    SetDatum.srv
    SetPose.srv
    ToggleFilterProcessing.srv
)

generate_messages(
  DEPENDENCIES
    geographic_msgs
    geometry_msgs
    std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES 
    ekf
    filter_base
    filter_utilities
    navsat_transform
    ros_filter
    ros_filter_utilities
    robot_localization_estimator
    ros_robot_localization_listener
    ukf
  CATKIN_DEPENDS
    cmake_modules
    diagnostic_msgs
    diagnostic_updater
    eigen_conversions
    geographic_msgs
    geometry_msgs
    message_filters
    message_runtime
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
    xmlrpcpp
  DEPENDS
    ${EIGEN_PACKAGE}
    YAML_CPP
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

# Library definitions
add_library(filter_utilities src/filter_utilities.cpp)
add_library(filter_base src/filter_base.cpp)
add_library(ekf src/ekf.cpp)
add_library(ukf src/ukf.cpp)
add_library(robot_localization_estimator src/robot_localization_estimator.cpp)
add_library(ros_robot_localization_listener src/ros_robot_localization_listener.cpp)
add_library(ros_filter_utilities src/ros_filter_utilities.cpp)
add_library(ros_filter src/ros_filter.cpp)
add_library(navsat_transform src/navsat_transform.cpp)

# Executables
add_executable(ekf_localization_node src/ekf_localization_node.cpp)
add_executable(ukf_localization_node src/ukf_localization_node.cpp)
add_executable(navsat_transform_node src/navsat_transform_node.cpp)
add_executable(robot_localization_listener_node src/robot_localization_listener_node.cpp)

# Dependencies
add_dependencies(filter_base ${PROJECT_NAME}_gencpp)
add_dependencies(navsat_transform ${PROJECT_NAME}_gencpp)
add_dependencies(robot_localization_listener_node ${PROJECT_NAME}_gencpp)

# Linking
target_link_libraries(ros_filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(filter_base filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(ekf filter_base ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(ukf filter_base ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(ros_filter ekf ukf ros_filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(robot_localization_estimator filter_utilities filter_base ekf ukf ${EIGEN3_LIBRARIES})
target_link_libraries(ros_robot_localization_listener robot_localization_estimator ros_filter_utilities
  ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${YAML_CPP_LIBRARIES})
target_link_libraries(robot_localization_listener_node ros_robot_localization_listener ${catkin_LIBRARIES})
target_link_libraries(ekf_localization_node ros_filter ${catkin_LIBRARIES})
target_link_libraries(ukf_localization_node ros_filter ${catkin_LIBRARIES})
target_link_libraries(navsat_transform filter_utilities ros_filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(navsat_transform_node navsat_transform ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
  ekf
  ekf_localization_node
  filter_base
  filter_utilities
  navsat_transform
  navsat_transform_node
  ros_filter
  ros_filter_utilities
  robot_localization_estimator
  ros_robot_localization_listener
  robot_localization_listener_node
  ukf
  ukf_localization_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch")

install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)

install(FILES
  LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)

  # Not really necessary, but it will cause the build to fail if it's
  # missing, rather than failing once the tests are being executed
  find_package(rosbag REQUIRED)
  find_package(rostest REQUIRED)

  #### FILTER BASE TESTS ####
  catkin_add_gtest(filter_base-test test/test_filter_base.cpp)
  target_link_libraries(filter_base-test filter_base ${catkin_LIBRARIES})

  # This test uses ekf_localization node for convenience.
  add_rostest_gtest(test_filter_base_diagnostics_timestamps
                    test/test_filter_base_diagnostics_timestamps.test
                    test/test_filter_base_diagnostics_timestamps.cpp)
  target_link_libraries(test_filter_base_diagnostics_timestamps ${catkin_LIBRARIES} ${rostest_LIBRARIES})
  add_dependencies(test_filter_base_diagnostics_timestamps ekf_localization_node)

  #### EKF TESTS #####
  add_rostest_gtest(test_ekf
                    test/test_ekf.test
                    test/test_ekf.cpp)
  target_link_libraries(test_ekf ros_filter ekf ${catkin_LIBRARIES} ${rostest_LIBRARIES})


  add_rostest_gtest(test_ekf_localization_node_interfaces
                    test/test_ekf_localization_node_interfaces.test
                    test/test_ekf_localization_node_interfaces.cpp)
  target_link_libraries(test_ekf_localization_node_interfaces ${catkin_LIBRARIES} ${rostest_LIBRARIES})

  add_rostest_gtest(test_ekf_localization_node_bag1
                    test/test_ekf_localization_node_bag1.test
                    test/test_localization_node_bag_pose_tester.cpp)
  target_link_libraries(test_ekf_localization_node_bag1 ${catkin_LIBRARIES} ${rostest_LIBRARIES})

  add_rostest_gtest(test_ekf_localization_node_bag2
                    test/test_ekf_localization_node_bag2.test
                    test/test_localization_node_bag_pose_tester.cpp)
  target_link_libraries(test_ekf_localization_node_bag2 ${catkin_LIBRARIES} ${rostest_LIBRARIES})

  add_rostest_gtest(test_ekf_localization_node_bag3
                    test/test_ekf_localization_node_bag3.test
                    test/test_localization_node_bag_pose_tester.cpp)
  target_link_libraries(test_ekf_localization_node_bag3 ${catkin_LIBRARIES} ${rostest_LIBRARIES})

  #### UKF TESTS #####
  add_rostest_gtest(test_ukf
                    test/test_ukf.test
                    test/test_ukf.cpp)
  target_link_libraries(test_ukf ros_filter ukf ${catkin_LIBRARIES} ${rostest_LIBRARIES})

  add_rostest_gtest(test_ukf_localization_node_interfaces
                    test/test_ukf_localization_node_interfaces.test
                    test/test_ukf_localization_node_interfaces.cpp)
  target_link_libraries(test_ukf_localization_node_interfaces ${catkin_LIBRARIES} ${rostest_LIBRARIES})

  add_rostest_gtest(test_ukf_localization_node_bag1
                    test/test_ukf_localization_node_bag1.test
                    test/test_localization_node_bag_pose_tester.cpp)
  target_link_libraries(test_ukf_localization_node_bag1 ${catkin_LIBRARIES} ${rostest_LIBRARIES})

  add_rostest_gtest(test_ukf_localization_node_bag2
                    test/test_ukf_localization_node_bag2.test
                    test/test_localization_node_bag_pose_tester.cpp)
  target_link_libraries(test_ukf_localization_node_bag2 ${catkin_LIBRARIES} ${rostest_LIBRARIES})

  add_rostest_gtest(test_ukf_localization_node_bag3
                    test/test_ukf_localization_node_bag3.test
                    test/test_localization_node_bag_pose_tester.cpp)
  target_link_libraries(test_ukf_localization_node_bag3 ${catkin_LIBRARIES} ${rostest_LIBRARIES})

  #### RLE/RLL TESTS #####
  add_rostest_gtest(test_robot_localization_estimator
                    test/test_robot_localization_estimator.test
                    test/test_robot_localization_estimator.cpp)
  target_link_libraries(test_robot_localization_estimator
                        robot_localization_estimator
                        ${catkin_LIBRARIES}
                        ${rostest_LIBRARIES})

  add_executable(test_ros_robot_localization_listener_publisher test/test_ros_robot_localization_listener_publisher.cpp)
  target_link_libraries(test_ros_robot_localization_listener_publisher
                        ${catkin_LIBRARIES})

  add_rostest_gtest(test_ros_robot_localization_listener
                    test/test_ros_robot_localization_listener.test
                    test/test_ros_robot_localization_listener.cpp)
  target_link_libraries(test_ros_robot_localization_listener
                        ros_robot_localization_listener
                        ${catkin_LIBRARIES}
                        ${rostest_LIBRARIES})

endif()
