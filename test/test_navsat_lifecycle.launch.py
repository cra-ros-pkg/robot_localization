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

import os
import sys
import pathlib
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_testing.legacy import LaunchTestService
from launch import LaunchService
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    parameters_file_dir = pathlib.Path(__file__).resolve().parent
    parameters_file_path = parameters_file_dir / 'test_navsat_transform.yaml'
    navsat_node = launch_ros.actions.LifecycleNode(
        package='robot_localization',
        executable='navsat_transform_node',
        name='test_navsat_lifecycle_node',
        namespace='',
        autostart=False, # We keep this false so the test can trigger transitions
        output='screen',
        parameters=[parameters_file_path],)
    return LaunchDescription([
        navsat_node,
    ])

def main(argv=sys.argv[1:]):
    ld = generate_launch_description()
    test_action = ExecuteProcess(
        cmd=[get_package_prefix('robot_localization') + '/lib/robot_localization/test_navsat_transform_lifecycle'],
        output='screen',
    )
    lts = LaunchTestService()
    lts.add_test_action(ld, test_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)

if __name__ == '__main__':
    sys.exit(main())