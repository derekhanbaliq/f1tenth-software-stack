#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.  # v_x
        self.thres = 1.5
        self.ackermann_msg = AckermannDriveStamped()
        # create ROS subscribers and publishers.
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_scan  # prevent unused variable warning
        self.sub_odom = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.sub_odom  # prevent unused variable warning
        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def odom_callback(self, odom_msg):
        # update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # calculate TTC
        r = np.array(scan_msg.ranges)
        theta = np.linspace(scan_msg.angle_min, scan_msg.angle_max, 1080)
        r_dot = self.speed * np.cos(theta)  # v_x projection & range rate are different in definition, but numerically equivalent
        ttc = r / np.clip(r_dot, a_min=0.001, a_max=None)  # 0.001 reduces inf & nan
        min_ttc = np.min(np.clip(ttc, 0.0, 60.0))  # clip ittc between 0 ~ 60s
        # publish command to brake
        if (self.speed > 0 and min_ttc < self.thres) or (self.speed < 0 and min_ttc < (self.thres + 0.8)):  # reversing should consider car length & blind spots
            print('min_ttc is {}, brake!!!!'.format(round(min_ttc, 2)))
            self.ackermann_msg.drive.speed = 0.0
            self.pub_drive.publish(self.ackermann_msg)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()