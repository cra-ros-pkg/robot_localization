#!/usr/bin/env python3
import launch
from launch import LaunchDescription
import launch_ros.actions
import yaml
import sys
from launch.substitutions import EnvironmentVariable
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch_testing.legacy import LaunchTestService
from launch.actions import ExecuteProcess
from launch import LaunchService
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    #*****test_navsat_transform.test*****
    navsat_transform = launch_ros.actions.Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='test_navsat_transform',
        output='screen',
    )

    return LaunchDescription([
        navsat_transform,
    ])


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    test1_action = ExecuteProcess(
        cmd=[get_package_prefix('robot_localization') + '/lib/robot_localization/test_navsat_transform'],
        output='screen',
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)

if __name__ == '__main__':
    sys.exit(main())
