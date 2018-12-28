#!/usr/bin/env python3

import os
import pathlib
import unittest
import rclpy
from unittest.mock import Mock
from time import sleep
from rclpy.task import Future
from rclpy.task import Task

from launch import LaunchDescription
from launch import LaunchService
from launch import LaunchIntrospector
from launch_ros import get_default_launch_description
from launch.substitutions import EnvironmentVariable
import launch_ros.actions.node
import yaml
import threading
import yaml
import launch.actions
from launch.actions import DeclareLaunchArgument

TEST_NODE = 'test_node'
TEST_NAMESPACE = '/'

class TestCPUMonitor(unittest.TestCase):
    @classmethod
    def test_test_ekf_localization_node_interfaces(self):
        print("test_ekf_localization_node_interfaces Testcases Execution Start...")
        os.system("ros2 launch robot_localization test_ekf_localization_node_interfaces.launch.py")

    @classmethod
    def test_test_ukf_localization_node_interfaces(self):
        print("test_ukf_localization_node_interfaces Testcases Execution Start...")
        os.system("ros2 launch robot_localization test_ukf_localization_node_interfaces.launch.py")

    @classmethod
    def test_filter_base(self):
        print("filter_base Testcases Execution Start...")
        os.system("ros2 run robot_localization filter_base-test")

    @classmethod
    def test_filter_base_diagnostics_timestamps(self):
        print("test_filter_base_diagnostics_timestamps Testcases Execution Start...")
        os.system("ros2 launch robot_localization test_filter_base_diagnostics_timestamps.launch.py")

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(TEST_NODE, namespace=TEST_NAMESPACE)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _assert_launch_errors(self, actions):
        ld = LaunchDescription(actions)
        self.ls = LaunchService()
        self.ls.include_launch_description(ld)
        assert 0 != self.ls.run()

    def _assert_launch_no_errors(self, actions):
        ld = LaunchDescription(actions)
        self.ls = LaunchService()
        self.ls.include_launch_description(ld)
        t = threading.Thread(target=self.ls.run, kwargs={'shutdown_when_idle': False})
        t.start()

if __name__ == '__main__':
    unittest.main()


