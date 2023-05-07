import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from scipy.spatial import distance, transform
import os


import numpy as np
from math import pi, atan2, cos, sin
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PointStamped, QuaternionStamped, TransformStamped, PoseStamped

import torch
from torch import nn
from torch.distributions.normal import Normal

NUM_LIDAR_SCANS = 720//10

def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
    torch.nn.init.orthogonal_(layer.weight, std)
    torch.nn.init.constant_(layer.bias, bias_const)
    return layer

class Agent(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.critic = nn.Sequential(
            layer_init(nn.Linear(in_channels, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, 1), std=1.0),
        )
        self.actor_mean = nn.Sequential(
            layer_init(nn.Linear(in_channels, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, out_channels), std=0.01),
        )
        self.actor_logstd = nn.Parameter(torch.zeros(1, out_channels))

    def get_value(self, x):
        return self.critic(x)

    def get_action_and_value(self, x, action=None):
        action_mean = self.actor_mean(x)
        action_logstd = self.actor_logstd.expand_as(action_mean)
        action_std = torch.exp(action_logstd)
        probs = Normal(action_mean, action_std)
        if action is None:
            action = probs.sample()
        return action, probs.log_prob(action).sum(1), probs.entropy().sum(1), self.critic(x)



class RLPlanner(Node):
    def __init__(self):
        super().__init__('RlPlanner')

        self.T = 1

        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        odom_topic = '/pf/viz/inferred_pose'

        # PURE PURSUIT
        self.is_ascending = True  # waypoint indices are ascending during tracking
        self.map_name = 'levine_centerline'

        # Subscribe to POSE
        self.sub_pose = self.create_subscription(PoseStamped, odom_topic, self.pose_callback, 1)
        self.currPos = np.array([0.0, 0.0]).reshape((1, 2))

        map_path = os.path.abspath(os.path.join('levine'))
        csv_data = np.loadtxt(map_path + '/' + self.map_name + '.csv', delimiter=';', skiprows=0)  # csv data
        self.waypoints = csv_data[:, 1:3]  # first row is indices
        self.numWaypoints = self.waypoints.shape[0]
        self.ref_speed = 1.0 * np.ones_like(csv_data[:, 5])

        # params for levine 2nd - real
        self.L = 2.2
        self.steering_gain = 0.45


        # RL PLANNER
        # create subscribers and publishers
        self.ackermann_msg = AckermannDriveStamped()
        self.ackermann_msg.drive.speed = 0.
        self.ackermann_msg.drive.steering_angle = 0.
        
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.drive_publisher.publish(self.ackermann_msg)
        self.scan_subscriber = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)

        # import the model we trained(""
        
        self.agent = Agent(NUM_LIDAR_SCANS, self.T)
        model_path = "ppo_with_noise/noise_reward_370.pt"
        # model_path = "33_model.pt"
        model = torch.load(model_path)
        self.agent.load_state_dict(model["model_state_dict"])
    
    # PURE PURSUIT
    def pose_callback(self, pose_msg):
        
        # Get current pose
        self.currX = pose_msg.pose.position.x
        self.currY = pose_msg.pose.position.y
        self.currPos = np.array([self.currX, self.currY]).reshape((1, 2))
        print("currPos =", self.currPos)

        # Transform quaternion pose message to rotation matrix
        quat = pose_msg.pose.orientation
        self.quat = [quat.x, quat.y, quat.z, quat.w]
        R = transform.Rotation.from_quat(self.quat)
        self.rot = R.as_matrix()

        # Find closest waypoint to where we are
        self.distances = distance.cdist(self.currPos, self.waypoints, 'euclidean').reshape((self.numWaypoints))
        self.closest_index = np.argmin(self.distances)
        self.closestPoint = self.waypoints[self.closest_index]

        # Find target point
        self.targetPoint, self.targetPointIdx = self.get_closest_point_beyond_lookahead_dist(self.L)

        # Homogeneous transformation
        translatedTargetPoint = self.translatePoint(self.targetPoint)
        
        # calculate curvature/steering angle
        y = translatedTargetPoint[1]
        gamma = self.steering_gain * (2 * y / self.L**2)
        gamma = np.clip(gamma, -0.35, 0.35)
        
    def get_closest_point_beyond_lookahead_dist(self, threshold):
        point_index = self.closest_index
        dist = self.distances[point_index]
        
        while dist < threshold:
            if self.is_ascending:
                point_index += 1
                if point_index >= len(self.waypoints):
                    point_index = 0
                dist = self.distances[point_index]
            else:
                point_index -= 1
                if point_index < 0:
                    point_index = len(self.waypoints) - 1
                dist = self.distances[point_index]

        point = self.waypoints[point_index]

        return point, point_index

    def translatePoint(self, targetPoint):
        H = np.zeros((4, 4))
        H[0:3, 0:3] = np.linalg.inv(self.rot)
        H[0, 3] = self.currX
        H[1, 3] = self.currY
        H[3, 3] = 1.0
        pvect = targetPoint - self.currPos
        convertedTarget = (H @ np.array((pvect[0, 0], pvect[0, 1], 0, 0))).reshape((4))
        
        return convertedTarget
    
    # RL PLANNER
    def scan_callback(self, scan_msg):
        scans = np.array(scan_msg.ranges[:-1]).flatten()
        print(scans.shape)

        def get_heading_from_quaternion(q):
            qx, qy, qz, qw = q
            # roll = np.arctan2(2.0 * (qz * qy + qw * qx), 1.0 - 2.0 * (qx * qx + qy * qy))
            # pitch = np.arcsin(2.0 * (qy * qw - qz * qx))
            heading = np.arctan2(2.0 * (qz * qw + qx * qy), -1.0 + 2.0 * (qw * qw + qx * qx))
            return heading # np.array([heading]).reshape(-1, 1)

        # target_points = []
        # for i in range(self.T):
        #     if i == 0:
        #         target_points.append(self.targetPoint.reshape(-1, 1))
        #     else:
        #         wp = self.waypoints[(self.targetPointIdx + i) % self.waypoints.shape[0]]
        #         # obs[start_idx:end_idx, :] = wp.reshape(-1, 1)
        #         target_points.append(wp.reshape(-1, 1))
        # target_points = np.concatenate(target_points)
        heading = get_heading_from_quaternion(self.quat)
        # pos = np.array([self.currPos[:, 0].item(), self.currPos[:, 1].item(), heading])
        # obs = np.concatenate((pos.reshape(-1, 1), scans.reshape(-1, 1), target_points))
        
        scans = scans[180:900]
        scans = scans[::10]
        # scans = np.clip(scans, 0, 30)
        

        obs = torch.from_numpy(scans).float()
        obs = obs.reshape(1, -1)
        # print(type(scans))
        action, _, _, _ = self.agent.get_action_and_value(obs)
        print(f"action {action}")
        action = action.numpy()

        # do our code here
        # if isinstance(offset, np.ndarray):  # agent == 1:
        R = lambda theta: np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        axis = np.array([0, 1]).reshape(-1, 1)
        rotated_offset = R(heading) @ axis * action[0]
        # import ipdb; ipdb.set_trace()

        targetPoint = self.targetPoint + rotated_offset[:, 0]  # += is not overwritten by np!

        # calculate steering angle / curvature
        waypoint_y = np.dot(np.array([np.sin(-heading), np.cos(-heading)]),
                            targetPoint - np.array([self.currX, self.currY]))
        gamma = self.steering_gain * 2.0 * waypoint_y / self.L ** 2
        steering_angle = gamma
        # radius = 1 / (2.0 * waypoint_y / self.L ** 2)
        # steering_angle = np.arctan(0.33 / radius)  # Billy's method, but it also involves tricky fixing
        steering_angle = np.clip(steering_angle, -0.35, 0.35)

        # calculate speed
        speed = self.ref_speed[self.targetPointIdx]


        self.ackermann_msg.drive.speed = -1 * speed # special car :-)
        self.ackermann_msg.drive.steering_angle = steering_angle
        print("steering =", self.ackermann_msg.drive.steering_angle)
        print("speed =", self.ackermann_msg.drive.speed)

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
