# Copyright 2018 Open Source Robotics Foundation, Inc.
#
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


def generate_launch_description():
    parameters_file_dir = pathlib.Path(__file__).resolve().parent
    parameters_file_path = parameters_file_dir /'test_ekf_localization_node_bag1.yaml'
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'output_final_position',
            default_value='false'),
        launch.actions.DeclareLaunchArgument(
            'output_location',
	    default_value='ekf1.txt'),
	
	#launch_ros.actions.Node(
         #   package='tf2_ros', node_executable='static_transform_publisher',node_name='bl_imu', output='screen',                       
          #  arguments=['0', '-0.3', '0.52', '-1.570796327', '0', '1.570796327', 'base_link', 'imu_link']            		
           #),

	launch_ros.actions.Node(
            package='robot_localization', executable='ekf_node', name='test_ekf_localization_node_bag1_ekf',
	    output='screen',
            parameters=[
                parameters_file_path,
                str(parameters_file_path),
                [EnvironmentVariable(name='FILE_PATH'), os.sep, 'test_ekf_localization_node_bag1.yaml'],
           ],
           ),
        
        launch_ros.actions.Node(
            package='robot_localization', executable='test_ekf_localization_node_bag1', name='test_ekf_localization_node_bag1_pose',
            output='screen',
	    arguments=['--time-limit','10'],
	    parameters=[
                parameters_file_path,
                str(parameters_file_path),
                [EnvironmentVariable(name='FILE_PATH'), os.sep, 'test_ekf_localization_node_bag1.yaml'],
           ],
           ),	
])
