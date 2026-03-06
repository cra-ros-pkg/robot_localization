#!/usr/bin/env python3
"""
Dummy odometry publisher for integration tests.
Publishes static odometry at 10 Hz.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class TestOdomPublisher(Node):
    def __init__(self):
        super().__init__('test_odom_publisher')

        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.publish_odom)  # 10 Hz
        self.get_logger().info('Test odometry publisher started (10 Hz)')

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Static position at origin
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # Zero velocity
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.angular.z = 0.0

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = TestOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
