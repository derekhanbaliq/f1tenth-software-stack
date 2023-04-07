#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from scipy.spatial import distance, transform

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped, QuaternionStamped, TransformStamped, PoseStamped
import tf2_ros
from rclpy.time import Time, Duration
from scipy.interpolate import splprep, splev

import os


class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.is_real = False
        self.is_clockwise = False
        self.map_name = 'levine_2nd'

        # Topics & Subs, Pubs
        drive_topic = '/drive'
        odom_topic = '/pf/viz/inferred_pose' if self.is_real else '/ego_racecar/odom'
        visualization_topic = '/visualization_marker_array'

        # Subscribe to POSE
        self.sub_pose = self.create_subscription(PoseStamped if self.is_real else Odometry, odom_topic, self.pose_callback, 1)
        
        # Publish to drive
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.drive_msg = AckermannDriveStamped()

        # Publish to visualization
        self.pub_vis = self.create_publisher(MarkerArray, visualization_topic, 1)

        # Green
        self.waypointMarker = Marker()
        self.waypointMarker.header.frame_id = 'map'
        self.waypointMarker.type = Marker.POINTS
        self.waypointMarker.color.g = 1.0
        self.waypointMarker.color.a = 1.0
        self.waypointMarker.scale.x = 0.05
        self.waypointMarker.scale.y = 0.05
        self.waypointMarker.id = 0

        # Red
        self.targetMarker = Marker()
        self.targetMarker.header.frame_id = 'map'
        self.targetMarker.type = Marker.POINTS
        self.targetMarker.color.r = 1.0
        self.targetMarker.color.a = 1.0
        self.targetMarker.scale.x = 0.2
        self.targetMarker.scale.y = 0.2
        self.targetMarker.id = 1

        # Blue
        self.closestMarker = Marker()
        self.closestMarker.header.frame_id = 'map'
        self.closestMarker.type = Marker.POINTS
        self.closestMarker.color.b = 1.0
        self.closestMarker.color.a = 1.0
        self.closestMarker.scale.x = 0.2
        self.closestMarker.scale.y = 0.2
        self.closestMarker.id = 2
        
        # Cyan
        self.speedMarker = Marker()
        self.speedMarker.header.frame_id = 'map'
        self.speedMarker.type = Marker.POINTS
        self.speedMarker.color.b = 1.0
        self.speedMarker.color.g = 1.0
        self.speedMarker.color.a = 1.0
        self.speedMarker.scale.x = 0.2
        self.speedMarker.scale.y = 0.2
        self.speedMarker.id = 3

        map_path = os.path.abspath(os.path.join('src', 'pure_pursuit', 'map_data'))
        csv_data = np.loadtxt(map_path + '/' + self.map_name + '.csv', delimiter=';', skiprows=0)  # csv data
        self.waypoints = csv_data[:, 1:3]  # first row is indices
        self.numWaypoints = self.waypoints.shape[0]
        self.waypointMarker.points = [Point(x = wpt[0], y = wpt[1], z = 0.0) for wpt in self.waypoints]

        self.L = 2.0
        self.threshold = 0.01
        self.steeringGain = 1.0

    def pose_callback(self, pose_msg):
        
        # Get current pose
        self.currX = pose_msg.pose.position.x if self.is_real else pose_msg.pose.pose.position.x
        self.currY = pose_msg.pose.position.y if self.is_real else pose_msg.pose.pose.position.y
        self.currPos = np.array([self.currX, self.currY]).reshape((1, 2))

        # Transform quaternion pose message to rotation matrix
        quat = pose_msg.pose.orientation if self.is_real else pose_msg.pose.pose.orientation
        quat = [quat.x, quat.y, quat.z, quat.w]
        R = transform.Rotation.from_quat(quat)
        self.rot = R.as_matrix()

        # Find closest waypoint to where we are
        self.distances = distance.cdist(self.currPos, self.waypoints, 'euclidean').reshape((self.numWaypoints))
        self.closestPoint = self.waypoints[np.argmin(self.distances)]

        # Find target point
        targetPoint = self.get_closest_point_beyond_lookahead_dist(self.L)

        # Homogeneous transformation
        translatedTargetPoint = self.translatePoint(targetPoint)
        
        # calculate curvature/steering angle
        y = translatedTargetPoint[1]
        gamma = self.steeringGain * (2 * y / self.L**2)

        # publish drive message, don't forget to limit the steering angle.
        gamma = np.clip(gamma, -0.35, 0.35)
        self.drive_msg.drive.steering_angle = gamma
        self.drive_msg.drive.speed = (-1.0 if self.is_real else 1.0) * self.getVelocity()
        self.pub_drive.publish(self.drive_msg)

        # Visualizing points
        self.targetMarker.points = [Point(x = targetPoint[0], y = targetPoint[1], z = 0.0)]
        self.closestMarker.points = [Point(x = self.closestPoint[0], y = self.closestPoint[1], z = 0.0)]

        markerArray = MarkerArray()
        markerArray.markers = [self.waypointMarker, self.targetMarker, self.closestMarker, self.speedMarker]

        self.pub_vis.publish(markerArray)
        
    def get_closest_point_beyond_lookahead_dist(self, threshold):

        point_index = np.argmin(self.distances)
        dist = self.distances[point_index]
        while dist < threshold:
            if self.is_clockwise:
                point_index -= 1
                if point_index < 0:
                    point_index = len(self.waypoints) - 1
                dist = self.distances[point_index]
            else:
                point_index += 1
                if point_index >= len(self.waypoints):
                    point_index = 0
                dist = self.distances[point_index]

        point = self.waypoints[point_index]

        return point

    def getVelocity(self):
        minV = 4.0
        nominalV = 8.0

        speedTarget = self.get_closest_point_beyond_lookahead_dist(1.2 * self.L)
        translatedSpeedTarget = self.translatePoint(speedTarget)

        self.speedMarker.points = [Point(x = speedTarget[0], y = speedTarget[1], z = 0.0)]

        theta = np.arctan2(translatedSpeedTarget[1], translatedSpeedTarget[0])
        theta = np.abs(theta)
        scaler = 1 - theta / (np.pi / 2.0)
        velocity = scaler * nominalV
        # print(theta / (np.pi / 2.0))
        print(round(velocity, 2))
        
        return np.clip(velocity, minV, nominalV)

    def translatePoint(self, targetPoint):
        H = np.zeros((4, 4))
        H[0:3, 0:3] = np.linalg.inv(self.rot)
        H[0, 3] = self.currX
        H[1, 3] = self.currY
        H[3, 3] = 1.0
        pvect = targetPoint - self.currPos
        convertedTarget = (H @ np.array((pvect[0, 0], pvect[0, 1], 0, 0))).reshape((4))
        
        return convertedTarget


def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()