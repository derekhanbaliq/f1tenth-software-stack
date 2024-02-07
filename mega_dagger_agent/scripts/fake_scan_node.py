#!/usr/bin/env python3

"""
    MEGA-DAgger Fake Scan
    Author: Derek Zhou
    References: https://github.com/derekhanbaliq/f1tenth-software-stack
                https://github.com/M4D-SC1ENTIST/MEGA-DAgger/blob/main/f1tenth/imitation_learning/es_src/scripts/test_data/0208_supplementary.ipynb
"""

import numpy as np
import math
from scipy.spatial import transform
import laser_models
from numba import njit
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped, QuaternionStamped, TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster


class FakeScan(Node):
    """ 
    Implement Pure Pursuit on the car
    """
    def __init__(self):
        super().__init__('fake_scan_node')

        self.is_real = True

        # Topics & Subs, Pubs
        odom_topic = '/pf/viz/inferred_pose' if self.is_real else '/ego_racecar/odom'
        fake_lidar_scan_topic = '/fake_scan'
        # lidar_scan_topic = '/scan'  # for debugging fake_scan
        # visualization_topic = '/fake_scan_visualization_marker_array'

        # Subscribe to POSE
        self.sub_pose = self.create_subscription(PoseStamped if self.is_real else Odometry, odom_topic, self.pose_callback, 1)
        self.sub_pose
        # self.sub_scan = self.create_subscription(LaserScan, lidar_scan_topic, self.scan_callback, 5)
        # self.sub_scan  # prevent unused variable warning
        # Publish to fake scan
        self.pub_fake_scan = self.create_publisher(LaserScan, fake_lidar_scan_topic, 1)
        self.fake_scan_msg = LaserScan()
        # Publish to visualization
        # self.pub_vis = self.create_publisher(MarkerArray, visualization_topic, 1)
        # self.markerArray = MarkerArray()

        # Timer
        self.timer = self.create_timer(0.01, self.timer_callback)  # timer period = 0.01s

        # Init setup & parameters
        self.fake_scan_init()
        # self.visualization_init()

    def pose_callback(self, pose_msg):
        # Get current pose
        x = pose_msg.pose.position.x if self.is_real else pose_msg.pose.pose.position.x
        y = pose_msg.pose.position.y if self.is_real else pose_msg.pose.pose.position.y
        pos = np.array([x, y, 0.0])
        print('position = ', pos)

        # Transform quaternion pose message to rotation matrix
        q = pose_msg.pose.orientation if self.is_real else pose_msg.pose.pose.orientation
        q = [q.x, q.y, q.z, q.w]
        # R = transform.Rotation.from_quat(q).as_matrix()
        yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
        print('yaw = {}'.format(yaw))

        # Use laser_models to get scan from position and theta
        ego_scan = self.scan_sim.scan([x, y, yaw], None).flatten()  # this is not pos!
        # print('ego_scan = ', ego_scan)

        # vertices = np.asarray([[2.3, 0.65],[1.7, 0.65],[1.7, 0.45],[12.3, 0.45]])
        # self.ego_scan = laser_models.ray_cast(self.pos, 10. * np.ones((1080, )), np.linspace(-2.35, 2.35, num=1080), vertices)

        # Load fake scan data
        self.fake_scan_msg.ranges = ego_scan.tolist()
        # self.pub_fake_scan.publish(self.fake_scan_msg)

    def timer_callback(self):
        timestamp = self.get_clock().now().to_msg()

        # Publish fake scan data
        self.fake_scan_msg.header.stamp = timestamp
        self.pub_fake_scan.publish(self.fake_scan_msg)

    def scan_callback(self, scan_msg):
        # Check scan and fake scan
        scan = np.array(scan_msg.ranges).flatten()
        print('scan  =', scan[:10])
        fake_scan = np.array(self.fake_scan_msg.ranges).flatten()
        print('fake_scan  =', fake_scan[:10])
        
        plt.plot(np.arange(1080), scan, 'b')
        plt.plot(np.arange(1080), fake_scan, 'r')

        plt.show()
        plt.close()

    def fake_scan_init(self):
        self.fake_scan_msg.header.frame_id = '/ego_racecar/fake_scan'
        self.fake_scan_msg.angle_min = -4.7 / 2.
        self.fake_scan_msg.angle_max = 4.7 / 2.
        self.fake_scan_msg.angle_increment = 4.7 / 1080
        self.fake_scan_msg.range_min = 0.
        self.fake_scan_msg.range_max = 30.

        self.scan_sim = laser_models.ScanSimulator2D(1080, 4.7)
        self.scan_sim.set_map('/home/derek/sim_ws/src/mega_dagger_agent/maps/skir_mega_dagger.yaml', '.pgm')

    def visualization_init(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    print("Fake Scan Initialized")
    fake_scan_node = FakeScan()
    rclpy.spin(fake_scan_node)

    fake_scan_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
