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


class RLPlanner(Node):
    def __init__(self):
        super().__init__('RlPlanner')

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
        self.ref_speed = csv_data[:, 5]

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

        # import the model we trained
    
    # PURE PURSUIT
    def pose_callback(self, pose_msg):
        
        # Get current pose
        self.currX = pose_msg.pose.position.x
        self.currY = pose_msg.pose.position.y
        self.currPos = np.array([self.currX, self.currY]).reshape((1, 2))
        print("currPos =", self.currPos)

        # Transform quaternion pose message to rotation matrix
        quat = pose_msg.pose.orientation
        quat = [quat.x, quat.y, quat.z, quat.w]
        R = transform.Rotation.from_quat(quat)
        self.rot = R.as_matrix()

        # Find closest waypoint to where we are
        self.distances = distance.cdist(self.currPos, self.waypoints, 'euclidean').reshape((self.numWaypoints))
        self.closest_index = np.argmin(self.distances)
        self.closestPoint = self.waypoints[self.closest_index]

        # Find target point
        targetPoint = self.get_closest_point_beyond_lookahead_dist(self.L)

        # Homogeneous transformation
        translatedTargetPoint = self.translatePoint(targetPoint)
        
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

        return point

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
        # obs = np.concatenate((self.currPos, scans, self.waypoints))

        # print(type(scans))
        # action, _, _, _ = self.agent.get_action_and_value(obs)

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
