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
from numba import njit
import matplotlib.pyplot as plt
import laser_models
import collision_models

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

        self.is_real = False

        # Topics & Subs, Pubs
        odom_topic = '/pf/viz/inferred_pose' if self.is_real else '/ego_racecar/odom'
        oppo_odom_topic = '/oppo_odom' if self.is_real else '/ego_racecar/opp_odom'
        fake_lidar_scan_topic = '/fake_scan'
        # lidar_scan_topic = '/scan'  # for debugging fake_scan
        visualization_topic = '/fake_scan_visualization_marker_array'

        # Subscribe to pose
        self.sub_pose = self.create_subscription(PoseStamped if self.is_real else Odometry, odom_topic, self.pose_callback, 1)
        self.sub_pose  # prevent unused variable warning
        # Subscribe to oppo pose
        self.sub_oppo_pose = self.create_subscription(PoseStamped if self.is_real else Odometry, oppo_odom_topic, self.oppo_pose_callback, 1)
        self.sub_oppo_pose
        # Subscribe to fake lidar scan for debugging
        # self.sub_scan = self.create_subscription(LaserScan, lidar_scan_topic, self.scan_callback, 5)
        # self.sub_scan 
        
        # Publish to fake scan
        self.pub_fake_scan = self.create_publisher(LaserScan, fake_lidar_scan_topic, 1)
        self.fake_scan_msg = LaserScan()
        # Publish to visualization
        self.pub_vis = self.create_publisher(Marker, visualization_topic, 1)
        self.fake_scan_marker = Marker()

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # timer period = 0.01s

        # Init setup & parameters
        self.fake_scan_init()
        # self.visualization_init()

    def pose_callback(self, pose_msg):
        # Get current pose
        x = pose_msg.pose.position.x if self.is_real else pose_msg.pose.pose.position.x
        y = pose_msg.pose.position.y if self.is_real else pose_msg.pose.pose.position.y
        print('position = ', np.array([x, y]))

        # Transform quaternion pose message to rotation matrix
        q = pose_msg.pose.orientation if self.is_real else pose_msg.pose.pose.orientation
        q = [q.x, q.y, q.z, q.w]
        # R = transform.Rotation.from_quat(q).as_matrix()
        yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
        print('yaw = {}'.format(yaw))

        # Use laser_models to get scan from position and theta
        self.ego_pose = [x, y, yaw]
        self.ego_scan = self.F110ScanSimulator.scan(self.ego_pose, None).flatten()  # this is not pos!

    def oppo_pose_callback(self, pose_msg):
        # Get current pose
        x = pose_msg.pose.position.x if self.is_real else pose_msg.pose.pose.position.x
        y = pose_msg.pose.position.y if self.is_real else pose_msg.pose.pose.position.y
        print('oppo position = ', np.array([x, y]))

        # Transform quaternion pose message to rotation matrix
        q = pose_msg.pose.orientation if self.is_real else pose_msg.pose.pose.orientation
        q = [q.x, q.y, q.z, q.w]
        # R = transform.Rotation.from_quat(q).as_matrix()
        oppo_yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
        print('oppo yaw = {}'.format(oppo_yaw))

        # Update oppo car vertices
        self.oppo_vertices = collision_models.get_vertices(np.array([x + np.cos(oppo_yaw) * 0.275, y + np.sin(oppo_yaw) * 0.275, oppo_yaw]), 0.58, 0.31)
        print("oppo_vertices = ", self.oppo_vertices)

    def timer_callback(self):
        timestamp = self.get_clock().now().to_msg()

        # Get the blocked view scan
        blocked_scan_mask = laser_models.ray_cast(np.array(self.ego_pose), 20. * np.ones((1080, )), np.linspace(-2.35, 2.35, num=1080), self.oppo_vertices)  # with 10 m clip
        new_scan = np.minimum(self.ego_scan, blocked_scan_mask)  # get blocked scan elementwisely

        # Publish fake scan data
        self.fake_scan_msg.header.stamp = timestamp
        
        self.fake_scan_msg.ranges = new_scan.tolist()
        self.pub_fake_scan.publish(self.fake_scan_msg)
        print("publishing fake scan, 10 Hz")

        # Publish fake scan visualization
        # self.visualize_fake_scan()

    def scan_callback(self, scan_msg):
        # Check scan and fake scan
        scan = np.array(scan_msg.ranges).flatten()
        # print('scan  =', scan[:10])
        fake_scan = np.array(self.fake_scan_msg.ranges).flatten()
        # print('fake_scan  =', fake_scan[:10])
        
        plt.plot(np.arange(1080), scan, 'b', label='scan')
        plt.plot(np.arange(1080), fake_scan, 'r', label='fake scan')
        plt.legend(loc='upper right')

        plt.show()
        plt.close()

    def fake_scan_init(self):
        self.fake_scan_msg.header.frame_id = 'laser' if self.is_real else 'ego_racecar/laser'  # TODO: validate this
        self.fake_scan_msg.angle_min = -4.7 / 2.
        self.fake_scan_msg.angle_max = 4.7 / 2.
        self.fake_scan_msg.angle_increment = 4.7 / 1080
        self.fake_scan_msg.range_min = 0.
        self.fake_scan_msg.range_max = 30.

        self.F110ScanSimulator = laser_models.ScanSimulator2D(1080, 4.7)
        self.F110ScanSimulator.set_map('/home/derek/sim_ws/src/mega_dagger_agent/maps/skir_mega_dagger.yaml', '.pgm')
        
        self.ego_pose = [0.0, 0.0, 0.0]
        self.ego_scan = np.full(1080, 10)
        self.oppo_vertices = collision_models.get_vertices(np.array([2.0, 0.5, 0.0]), 0.58, 0.31)

    def visualization_init(self):
        # cyan
        self.fake_scan_marker.header.frame_id = 'map'
        self.fake_scan_marker.type = Marker.POINTS
        self.fake_scan_marker.color.g = 0.75
        self.fake_scan_marker.color.b = 0.75
        self.fake_scan_marker.color.a = 1.0
        self.fake_scan_marker.scale.x = 0.5
        self.fake_scan_marker.scale.y = 0.5
        self.fake_scan_marker.id = 1

    def visualize_fake_scan(self):
        self.fake_scan_marker.points = [Point(x = 1.0, y = 1.0, z = 1.0)]
        # print(len(self.fake_scan_msg.ranges))

        self.pub_vis.publish(self.fake_scan_marker)


def main(args=None):
    rclpy.init(args=args)
    print("Fake Scan Initialized")
    fake_scan_node = FakeScan()
    rclpy.spin(fake_scan_node)

    fake_scan_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
