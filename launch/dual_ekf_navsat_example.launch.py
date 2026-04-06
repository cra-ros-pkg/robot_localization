# Copyright 2018 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_localization_dir = get_package_share_directory('robot_localization')
    parameters_file_dir = os.path.join(robot_localization_dir, 'params')
    parameters_file_path = os.path.join(parameters_file_dir, 'dual_ekf_navsat_example.yaml')
    os.environ['FILE_PATH'] = str(parameters_file_dir)

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='Automatically configure and activate the dual-EKF stack. Set to false for managed lifecycle control.')
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    autostart = LaunchConfiguration('autostart')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        autostart_arg,
        namespace_arg,
        launch.actions.DeclareLaunchArgument(
            'output_final_position',
            default_value='false'),
        launch.actions.DeclareLaunchArgument(
            'output_location',
	    default_value='~/dual_ekf_navsat_example_debug.txt'),
	
    launch_ros.actions.LifecycleNode(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_odom',
            namespace=namespace,
            autostart=autostart,
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('odometry/filtered', 'odometry/local')]           
           ),
    launch_ros.actions.LifecycleNode(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_map',
            namespace=namespace,
            autostart=autostart,
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('odometry/filtered', 'odometry/global')]
           ),           
    launch_ros.actions.LifecycleNode(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform',
            namespace=namespace,
            autostart=autostart,
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('imu', 'imu/data'),
                        ('gps/fix', 'gps/fix'), 
                        ('gps/filtered', 'gps/filtered'),
                        ('odometry/gps', 'odometry/gps'),
                        ('odometry/filtered', 'odometry/global')]           

           )           
])
