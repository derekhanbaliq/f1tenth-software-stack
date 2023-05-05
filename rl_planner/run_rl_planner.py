import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np
from math import pi, atan2, cos, sin


class RLPlanner(Node):
    def __init__(self):
        super().__init__('RlPlanner')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # create subscribers and publishers
        self.ackermann_msg = AckermannDriveStamped()
        self.ackermann_msg.drive.speed = 0.
        self.ackermann_msg.drive.steering_angle = 0.
        
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.drive_publisher.publish(self.ackermann_msg)
        self.scan_subscriber = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)

        # import the model we trained

    def scan_callback(self, scan_msg):
        scans = np.array(scan_msg.ranges[:-1]).flatten()
        # print(type(scans))

        # do our code here

        self.ackermann_msg.drive.speed = 1.0
        self.ackermann_msg.drive.steering_angle = 0.0
        print("steering =", self.ackermann_msg.drive.speed)
        print("speed =", self.ackermann_msg.drive.steering_angle)

        self.drive_publisher.publish(self.ackermann_msg)


def main(args=None):
    rclpy.init(args=args)
    print("RL agent Initialized")
    rl_planner_node = RLPlanner()
    rclpy.spin(rl_planner_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rl_planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
