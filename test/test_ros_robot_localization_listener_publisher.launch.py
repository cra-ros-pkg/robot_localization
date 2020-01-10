#!/usr/bin/env python3

import ament_index_python.packages
import launch
import launch_ros.actions

def generate_launch_description():

  return launch.LaunchDescription([
    launch_ros.actions.Node(
      package='robot_localization',
      node_executable='test_ros_robot_localization_listener_publisher',
      remappings=[
        ('/odometry/filtered', 'robot_localization/odom/filtered'),
        ('/accel/filtered', 'robot_localization/acceleration/filtered'),
      ]
    ),
  ])
