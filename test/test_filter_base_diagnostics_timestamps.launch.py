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
    parameters_file_path = parameters_file_dir / 'test_filter_base_diagnostics_timestamps.yaml'    
    os.environ['FILE_PATH'] = str(parameters_file_dir)

    #*****test_filter_base_diagnostics_timestamps.test***** 
    se_node = launch_ros.actions.Node(
	    package='robot_localization', node_executable='se_node', node_name='ekf_localization',
	    output='screen',
	    parameters=[
		parameters_file_path,
		str(parameters_file_path),
		[EnvironmentVariable(name='FILE_PATH'), os.sep, 'test_filter_base_diagnostics_timestamps.yaml'],
	   ],)

    test_filter_base_diagnostics_timestamps = launch_ros.actions.Node(
            package='robot_localization', node_executable='test_filter_base_diagnostics_timestamps',node_name='test_filter_base_diagnostics',
            output='screen',
        parameters=[
                parameters_file_path,
                str(parameters_file_path),
                [EnvironmentVariable(name='FILE_PATH'), os.sep, 'test_filter_base_diagnostics_timestamps.yaml'],
           ],)
    return LaunchDescription([
        se_node,
        test_filter_base_diagnostics_timestamps,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=test_filter_base_diagnostics_timestamps,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )),
])
