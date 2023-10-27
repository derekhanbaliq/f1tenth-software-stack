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
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Subscribe to scan
        self.sub_scan = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.sub_scan  # prevent unused variable warning
        # Publish to drive
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.drive_msg = AckermannDriveStamped()

        # MEGA-DAgger config
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.agent = AgentPolicyMLP(observ_dim=108, hidden_dim=256, action_dim=2, lr=0.001, device=device)
        model_path = '/home/derek/sim_ws/src/mega_dagger_agent/models/mega_dagger.pkl'
        self.agent.load_state_dict(torch.load(model_path, map_location=device))

    def scan_callback(self, scan_msg):
        scan = np.array(scan_msg.ranges[::10]).flatten()  # 108
        # print(scan.shape)

        # NN input scan, output steering & speed
        agent_action = self.agent.get_action(scan)
        steering = float(agent_action[0])
        speed = float(agent_action[1])

        # publish drive message
        self.drive_msg.drive.steering_angle = steering
        self.drive_msg.drive.speed = (-1.0 if self.is_real else 1.0) * speed
        self.pub_drive.publish(self.drive_msg)
        print("steering = {}, speed = {}".format(round(steering, 5), round(speed, 5)))


def main(args=None):
    rclpy.init(args=args)
    print("MEGA-DAgger Agent Initialized")
    agent_node = MEGADAggerAgent()
    rclpy.spin(agent_node)

    agent_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

