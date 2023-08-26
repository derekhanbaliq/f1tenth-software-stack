#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        timer_period = 0.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.declare_parameter("v")
        self.declare_parameter("d")
        self.v = float(self.get_parameter("v").value)
        self.d = float(self.get_parameter("d").value)

    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.v
        msg.drive.steering_angle = self.d
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing speed {} and steering {}'.format(msg.drive.speed, round(msg.drive.steering_angle, 1)))


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
