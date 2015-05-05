cmake_minimum_required(VERSION 2.8.3)
project(robot_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  std_msgs
  geographic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  diagnostic_msgs
  diagnostic_updater
  cmake_modules
  message_generation)
find_package(Eigen REQUIRED)

add_definitions(-DEIGEN_NO_DEBUG)

###################################
## catkin specific configuration ##
###################################
add_service_files(
  FILES
  SetPose.srv
  SetDatum.srv
)

generate_messages(
  DEPENDENCIES
  geographic_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES filter_base ekf
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_ros
    message_filters
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    diagnostic_msgs
    diagnostic_updater
    cmake_modules
    message_runtime
  DEPENDS Eigen
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

# Library definition
add_library(filter_utilities src/filter_utilities.cpp)
add_library(filter_base src/filter_base.cpp)
add_library(ekf src/ekf.cpp)
add_library(ukf src/ukf.cpp)
add_library(ros_filter_utilities src/ros_filter_utilities.cpp)
add_library(ros_filter src/ros_filter.cpp)
add_library(navsat_transform src/navsat_transform.cpp)

# Dependencies
add_dependencies(filter_base robot_localization_gencpp)
add_dependencies(navsat_transform robot_localization_gencpp)

# Executables
add_executable(ekf_localization_node src/ekf_localization_node.cpp)
add_executable(ukf_localization_node src/ukf_localization_node.cpp)
add_executable(navsat_transform_node src/navsat_transform_node.cpp)

# Linking
target_link_libraries(ros_filter_utilities ${catkin_LIBRARIES})
target_link_libraries(filter_utilities ${catkin_LIBRARIES})
target_link_libraries(filter_base filter_utilities ${catkin_LIBRARIES})
target_link_libraries(ekf filter_base ${catkin_LIBRARIES})
target_link_libraries(ukf filter_base ${catkin_LIBRARIES})
target_link_libraries(ros_filter ekf ukf ros_filter_utilities ${catkin_LIBRARIES})
target_link_libraries(ekf_localization_node ros_filter ${catkin_LIBRARIES})
target_link_libraries(ukf_localization_node ros_filter ${catkin_LIBRARIES})
target_link_libraries(navsat_transform ${catkin_LIBRARIES})
target_link_libraries(navsat_transform_node navsat_transform ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
  filter_utilities
  filter_base
  ekf
  ukf
  ros_filter_utilities
  ros_filter
  ekf_localization_node
  ukf_localization_node
  navsat_transform
  navsat_transform_node
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
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".svn" EXCLUDE)

install(FILES
  LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

# Filter base unit tests
catkin_add_gtest(filter_base-test test/test_filter_base.cpp)
target_link_libraries(filter_base-test filter_base ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)

  # Not really necessary, but it will cause the build to fail if it's
  # missing, rather than failing once the tests are being executed
  find_package(rosbag REQUIRED)
  find_package(rostest REQUIRED)

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

endif()
