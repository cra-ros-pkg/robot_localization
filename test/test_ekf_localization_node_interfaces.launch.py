#!/usr/bin/env python3
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

"""Launch file for test_ekf_localization_node_interfaces."""

import launch
from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
import sys
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch_testing.legacy import LaunchTestService
from launch.actions import ExecuteProcess
from launch import LaunchService
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    parameters_file_dir = pathlib.Path(__file__).resolve().parent
    parameters_file_path = parameters_file_dir / 'test_ekf_localization_node_interfaces.yaml'    
    os.environ['FILE_PATH'] = str(parameters_file_dir)

	 #*****test_ekf_localization_node_interfaces.test***** 
    ekf_node = launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='test_ekf_localization_node_interfaces_ekf',
	        output='screen',
            parameters=[
                parameters_file_path,
                str(parameters_file_path),
                [EnvironmentVariable(name='FILE_PATH'), os.sep, 'test_ekf_localization_node_interfaces.yaml'],
           ],)

    return LaunchDescription([
        ekf_node,
    ])


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    test1_action = ExecuteProcess(
        cmd=[get_package_prefix('robot_localization') + '/lib/robot_localization/test_ekf_localization_node_interfaces'],
        output='screen',
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)

if __name__ == '__main__':
    sys.exit(main())
