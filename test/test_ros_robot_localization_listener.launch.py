#!/usr/bin/env python3

import ament_index_python.packages
import launch
import launch_ros.actions

import os

def generate_launch_description():

  default_params_yaml = os.path.join(
    ament_index_python.packages.get_package_share_directory('robot_localization'),
    'test', 'test_ros_robot_localization_listener.yaml')

  return launch.LaunchDescription([
    # TODO: Port ROS 1 test launch params: clear_params="true"
    launch_ros.actions.Node(
      package='robot_localization',
      executable='test_ros_robot_localization_listener',
      remappings=[('test_ros_robot_localization_listener', 'test_estimator')],
      arguments=['__params:=' + default_params_yaml],
      output='screen',
    ),
    # TODO: Port ROS 1 test launch params: clear_params="true"
    launch_ros.actions.Node(
      package='robot_localization',
      executable='test_ros_robot_localization_listener_publisher',
      name='test_estimator',
      remappings=[
        ('/odometry/filtered', 'odom/filtered'),
        ('/accel/filtered', 'acceleration/filtered'),
      ],
      output='screen',
    ),
  ])
