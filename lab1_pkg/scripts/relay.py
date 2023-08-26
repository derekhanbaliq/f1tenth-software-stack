#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('relay')
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.v = None
        self.d = None

    def listener_callback(self, msg):
        self.get_logger().info('speed {} and steering {}'.format(msg.drive.speed,round(msg.drive.steering_angle, 1)))
        self.v = msg.drive.speed
        self.d = msg.drive.steering_angle
        
    def timer_callback(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.v * 3
        msg.drive.steering_angle = self.d * 3
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing speed {} and steering {}'.format(msg.drive.speed, round(msg.drive.steering_angle, 1)))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
