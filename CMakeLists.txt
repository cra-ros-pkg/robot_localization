cmake_minimum_required(VERSION 2.8.3)
project(robot_localization)

find_package(catkin REQUIRED COMPONENTS roscpp tf message_filters std_msgs geometry_msgs nav_msgs sensor_msgs cmake_modules gps_common)
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${gps_common_INCLUDE_DIRS}
  LIBRARIES filter_base ekf
  CATKIN_DEPENDS roscpp tf message_filters std_msgs geometry_msgs nav_msgs sensor_msgs cmake_modules
  DEPENDS Eigen
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${gps_common_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_library(filter_base src/filter_base.cpp)
add_library(ekf src/ekf.cpp)
add_library(navsat_transform src/navsat_transform.cpp)

add_executable(ekf_localization_node src/ekf_localization_node.cpp)
add_executable(utm_transform_node src/utm_transform_node.cpp)
add_executable(navsat_transform_node src/navsat_transform_node.cpp)

target_link_libraries(ekf filter_base)
target_link_libraries(ekf_localization_node ekf filter_base ${catkin_LIBRARIES})
target_link_libraries(utm_transform_node ${catkin_LIBRARIES})
target_link_libraries(navsat_transform ${catkin_LIBRARIES})
target_link_libraries(navsat_transform_node navsat_transform ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS filter_base ekf ekf_localization_node utm_transform_node navsat_transform_node
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

#############
## Testing ##
#############

# Unit tests
catkin_add_gtest(filter_base-test test/test_filter_base.cpp)
target_link_libraries(filter_base-test filter_base ${catkin_LIBRARIES})

catkin_add_gtest(ekf-test test/test_ekf.cpp)
target_link_libraries(ekf-test ekf filter_base ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(ekf_localization_node-test-interfaces
                    test/test_ekf_localization_node_interfaces.test
                    test/test_ekf_localization_node_interfaces.cpp)
  target_link_libraries(ekf_localization_node-test-interfaces ${catkin_LIBRARIES})

  add_rostest_gtest(ekf_localization_node-test-bag1
                    test/test_ekf_localization_node_test_bag1.test
                    test/test_ekf_localization_node_bag_pose_tester.cpp)
  target_link_libraries(ekf_localization_node-test-bag1 ${catkin_LIBRARIES})

  add_rostest_gtest(ekf_localization_node-test-bag2
                    test/test_ekf_localization_node_test_bag2.test
                    test/test_ekf_localization_node_bag_pose_tester.cpp)
  target_link_libraries(ekf_localization_node-test-bag2 ${catkin_LIBRARIES})

  add_rostest_gtest(ekf_localization_node-test-bag3
                    test/test_ekf_localization_node_test_bag3.test
                    test/test_ekf_localization_node_bag_pose_tester.cpp)
  target_link_libraries(ekf_localization_node-test-bag3 ${catkin_LIBRARIES})
endif()

