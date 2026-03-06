#!/usr/bin/env python3
"""
Integration test for EKF startup with use_sim_time.

Tests both scenarios for sim_time initialization deadlock:
1. CLOCK_FIRST=1:  Clock starts before EKF (happy path)
2. CLOCK_FIRST=0:  EKF starts before clock (worst case - triggers deadlock)

With the bug: Scenario 2 deadlocks in initialize()
With the fix: Both scenarios pass

Scenario selection is controlled by the CLOCK_FIRST environment variable.
CMakeLists.txt runs this test twice with different CLOCK_FIRST values.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import pathlib
import os
import sys
from launch_testing.legacy import LaunchTestService
from launch import LaunchService


# Common test setup - defined once to avoid duplication
TEST_DIR = pathlib.Path(__file__).resolve().parent

EKF_PARAMS = {
    'use_sim_time': True,
    'frequency': 30.0,
    'two_d_mode': True,
    'odom_frame': 'odom',
    'base_link_frame': 'base_link',
    'world_frame': 'odom',
    'odom0': '/odom',
    'odom0_config': [True, False, False,   # X, Y, Z position
                     False, False, False,   # roll, pitch, yaw
                     False, False, False,   # X, Y, Z velocity
                     False, False, False,   # roll, pitch, yaw velocity
                     False, False, False],  # X, Y, Z acceleration
}

# Reusable actions
clock_publisher = ExecuteProcess(
    cmd=['python3', os.path.join(TEST_DIR, 'test_clock_publisher.py')],
    name='clock_publisher',
    output='screen'
)

odom_publisher = ExecuteProcess(
    cmd=['python3', os.path.join(TEST_DIR, 'test_odom_publisher.py'),
         '--ros-args', '-p', 'use_sim_time:=true'],
    name='odom_publisher',
    output='screen'
)

ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    parameters=[EKF_PARAMS],
    output='screen'
)


def generate_launch_description_clock_first():
    """Scenario 1: Clock starts before EKF (happy path)"""
    return LaunchDescription([
        clock_publisher,  # Clock first
        odom_publisher,
        ekf_node,         # EKF last
    ])


def generate_launch_description_ekf_first():
    """Scenario 2: EKF starts before clock (deadlock trigger)"""
    return LaunchDescription([
        ekf_node,  # EKF first
        TimerAction(
            period=1.0,  # Clock delayed 1 second to ensure EKF initializes first
            actions=[clock_publisher, odom_publisher]
        ),
    ])


def main(argv=sys.argv[1:]):
    """
    Main test function - verifies EKF publishes /odometry/filtered.

    Test scenario is controlled by 'CLOCK_FIRST' environment variable:
    - CLOCK_FIRST=1 (default): Clock starts before EKF
    - CLOCK_FIRST=0: EKF starts before clock (harder test)
    """
    # Check environment variable to determine scenario
    clock_first = os.environ.get('CLOCK_FIRST', '1') == '1'

    if clock_first:
        print("[TEST] Scenario 1: Clock starts BEFORE EKF (happy path)")
        ld = generate_launch_description_clock_first()
    else:
        print("[TEST] Scenario 2: EKF starts BEFORE clock (deadlock trigger)")
        ld = generate_launch_description_ekf_first()

    # Test script that checks if /odometry/filtered is published
    test_script = os.path.join(TEST_DIR, 'test_ekf_startup_checker.py')

    # Launch checker after a delay
    test_action = TimerAction(
        period=2.0 if not clock_first else 1.0,  # Longer delay for scenario 2
        actions=[
            ExecuteProcess(
                cmd=['python3', test_script, '--ros-args', '-p', 'use_sim_time:=true'],
                output='screen',
            )
        ]
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
