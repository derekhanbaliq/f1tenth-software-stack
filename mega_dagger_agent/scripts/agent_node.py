#!/usr/bin/env python3

"""
    MEGA-DAgger Agent
    Author: Derek Zhou
    References: https://github.com/zzjun725/f1tenth_rl
                https://github.com/derekhanbaliq/RL-planner
"""

import math
import numpy as np
import os
from scipy.spatial import distance, transform

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

import torch
from agent_mlp import AgentPolicyMLP


class MEGADAggerAgent(Node):
    def __init__(self):
        super().__init__('agent_node')

        self.is_real = False

        # Topics & Subs, Pubs
        lidarscan_topic = '/fake_scan'  # /scan or /fake_scan
        drive_topic = '/drive'
        visualization_topic = '/visualization_marker_array'

        # Subscribe to scan
        self.sub_scan = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.sub_scan  # prevent unused variable warning
        # Publish to drive
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.drive_msg = AckermannDriveStamped()
        # Publish to visualization
        self.pub_vis = self.create_publisher(MarkerArray, visualization_topic, 1)
        self.markerArray = MarkerArray()

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)  # timer period = 0.01s

        # waypoints init
        self.visualization_init()

        # MEGA-DAgger config
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.agent = AgentPolicyMLP(observ_dim=108, hidden_dim=256, action_dim=2, lr=0.001, device=device)
        model_path = '/home/derek/sim_ws/src/mega_dagger_agent/models/skir2/mega_dagger.pkl'
        self.agent.load_state_dict(torch.load(model_path, map_location=device))

    def scan_callback(self, scan_msg):
        scan = np.array(scan_msg.ranges[::10]).flatten()  # 108
        if self.is_real:
            scan = scan[1:]
        # print(scan.shape)

        # NN input scan, output steering & speed
        agent_action = self.agent.get_action(scan)
        steering = float(agent_action[0])
        speed = min(float(agent_action[1]), 1.5)
        # print(speed)

        # publish drive message
        self.drive_msg.drive.steering_angle = steering
        self.drive_msg.drive.speed = (-1.0 if self.is_real else 1.0) * speed
        # self.pub_drive.publish(self.drive_msg)
        # print("steering = {}, speed = {}".format(round(steering, 5), round(speed, 5)))

    def timer_callback(self):
        self.pub_drive.publish(self.drive_msg)
        print("steering = {}, speed = {}".format(round(self.drive_msg.drive.steering_angle, 5), round(self.drive_msg.drive.speed, 5)))

    def visualization_init(self):
        traj_path = '/home/derek/sim_ws/src/mega_dagger_agent/trajs/'
        traj_data = np.loadtxt(traj_path + 'lane_2_traj_race_cl.csv', delimiter=';', skiprows=3)  # csv data
        optimal_traj_data = np.loadtxt(traj_path + 'lane_2_traj_race_cl_optimal.csv', delimiter=';', skiprows=3)

        # blue
        traj_data_marker = Marker()
        traj_data_marker.header.frame_id = 'map'
        traj_data_marker.type = Marker.POINTS
        traj_data_marker.color.b = 0.75
        traj_data_marker.color.a = 1.0
        traj_data_marker.scale.x = 0.05
        traj_data_marker.scale.y = 0.05
        traj_data_marker.id = 0
        traj_data_marker.points = [Point(x = wpt[0], y = wpt[1], z = 0.0) for wpt in traj_data[:, 1:3]]

        # cyan
        optimal_traj_data_marker = Marker()
        optimal_traj_data_marker.header.frame_id = 'map'
        optimal_traj_data_marker.type = Marker.POINTS
        optimal_traj_data_marker.color.g = 0.75
        optimal_traj_data_marker.color.b = 0.75
        optimal_traj_data_marker.color.a = 1.0
        optimal_traj_data_marker.scale.x = 0.05
        optimal_traj_data_marker.scale.y = 0.05
        optimal_traj_data_marker.id = 1
        optimal_traj_data_marker.points = [Point(x = wpt[0], y = wpt[1], z = 0.0) for wpt in optimal_traj_data[:, 1:3]]

        # red
        frame_marker = Marker()
        frame_marker.header.frame_id = 'map'
        frame_marker.type = Marker.LINE_STRIP
        frame_marker.color.r = 0.75
        frame_marker.color.a = 1.0
        frame_marker.scale.x = 0.1
        frame_marker.scale.y = 0.1
        frame_marker.id = 2
        frame_marker.points = [Point(x = 0.0, y = 0.0, z = 0.0), Point(x = 1.0, y = 0.0, z = 0.0), 
                                Point(x = 0.0, y = 0.0, z = 0.0), Point(x = 0.0, y = 1.0, z = 0.0),
                                Point(x = 0.0, y = 0.0, z = 0.0), Point(x = 0.0, y = 0.0, z = 1.0)]

        self.markerArray.markers = [traj_data_marker, optimal_traj_data_marker, frame_marker]
        self.pub_vis.publish(self.markerArray)


def main(args=None):
    rclpy.init(args=args)
    print("MEGA-DAgger Agent Initialized")
    agent_node = MEGADAggerAgent()
    rclpy.spin(agent_node)

    agent_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

