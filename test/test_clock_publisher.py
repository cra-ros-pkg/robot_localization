#!/usr/bin/env python3
"""
Simple clock publisher for integration tests.
Publishes /clock at 100 Hz to simulate Gazebo/rosbag playback.
"""
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class TestClockPublisher(Node):
    def __init__(self):
        super().__init__('test_clock_publisher')

        # Note: use_sim_time is automatically declared by ROS2, no need to declare it
        # This node uses wall time to publish the /clock topic

        self.publisher = self.create_publisher(Clock, '/clock', 10)
        self.timer = self.create_timer(0.01, self.publish_clock)  # 100 Hz
        self.sim_time = 0.0
        self.get_logger().info('Test clock publisher started (100 Hz)')

    def publish_clock(self):
        msg = Clock()
        self.sim_time += 0.01
        msg.clock.sec = int(self.sim_time)
        msg.clock.nanosec = int((self.sim_time % 1.0) * 1e9)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = TestClockPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
