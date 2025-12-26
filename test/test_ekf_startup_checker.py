#!/usr/bin/env python3
"""
Test checker script - verifies EKF publishes /odometry/filtered within timeout.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import sys
import time


class EKFStartupChecker(Node):
    def __init__(self):
        super().__init__('ekf_startup_checker')
        self.received = False

        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        self.get_logger().info('Waiting for /odometry/filtered...')

    def odom_callback(self, msg):
        self.get_logger().info('✓ Received /odometry/filtered - test PASSED')
        self.received = True


def main():
    rclpy.init()

    checker = EKFStartupChecker()

    # Wait up to 10 seconds (wall time) for EKF to publish
    # Using wall time because:
    # 1. We're testing node startup, not sim time behavior
    # 2. Sim clock may not be stable at test start
    timeout = 10.0
    start_time = time.time()

    while not checker.received and (time.time() - start_time) < timeout:
        rclpy.spin_once(checker, timeout_sec=0.1)

    checker.destroy_node()
    rclpy.shutdown()

    if checker.received:
        print('TEST PASSED: EKF started and published within timeout')
        return 0
    else:
        print('TEST FAILED: EKF did not publish within {}s (likely deadlock)'.format(timeout))
        return 1


if __name__ == '__main__':
    sys.exit(main())
