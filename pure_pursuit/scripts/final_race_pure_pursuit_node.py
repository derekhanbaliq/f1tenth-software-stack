#!/usr/bin/env python3

import numpy as np
from scipy.spatial import distance, transform
import os

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped, Quaternion, QuaternionStamped, TransformStamped, PoseStamped, Pose, PoseArray
import tf2_ros


class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.declare_params()

        self.is_real = self.get_parameter("sim or real").value
        self.is_ascending = self.get_parameter("is ascending").value  # waypoint indices are ascending during tracking
        self.csv_name = self.get_parameter("csv name").value

        # Topics & Subs, Pubs
        drive_topic = '/drive'
        odom_topic = '/pf/viz/inferred_pose' if self.is_real else '/ego_racecar/odom'
        visualization_topic = '/visualization_marker_array'
        pp_point_topic = '/pp_point'
        gf_point_topic = '/gf_point'
        vis_gf_marker_topic = '/vis_gf_marker'

        # Subscribe to POSE
        self.sub_pose = self.create_subscription(PoseStamped if self.is_real else Odometry, odom_topic, self.pose_callback, 1)
        self.sub_pose  # prevent unused variable warning
        # Publish to drive
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.drive_msg = AckermannDriveStamped()
        # Publish to visualization
        self.pub_vis = self.create_publisher(MarkerArray, visualization_topic, 1)
        self.markerArray = MarkerArray()
        
        # for motion planning
        # Publish goal point in car frame to reactive node
        self.pub_to_gf = self.create_publisher(PoseArray, pp_point_topic, 1)
        # Subscribe to gap following points
        self.sub_gf = self.create_subscription(Point, gf_point_topic, self.gap_following_callback, 1)
        self.sub_gf  # prevent unused variable warning
        
        # visualization of gap following point
        self.vis_gf_point_pub = self.create_publisher(Marker, vis_gf_marker_topic, 1)

        # loading waypoints
        csv_path = self.get_parameter("csv path").value
        csv_data = np.loadtxt(csv_path + '/' + self.csv_name + '.csv', delimiter=';', skiprows=0)  # csv data
        self.waypoints = csv_data[:, 1:3]  # first row is indices
        self.num_waypoints = self.waypoints.shape[0]
        self.ref_speed = csv_data[:, 5] * float(self.get_parameter("reference speed gain").value) # max speed - sim is 10m/s, levine 2nd - real is 6m/s

        self.visualization_init()

        # sim params
        self.L = float(self.get_parameter("lookahead distance").value)
        self.steering_gain = float(self.get_parameter("steering gain").value)
        self.test_speed = float(self.get_parameter("test speed").value)

    def declare_params(self):
        self.declare_parameter("sim or real")
        self.declare_parameter("is ascending")
        self.declare_parameter("csv name")
        self.declare_parameter("csv path")
        self.declare_parameter("reference speed gain")
        self.declare_parameter("lookahead distance")
        self.declare_parameter("steering gain")
        self.declare_parameter("test speed")

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
        self.distances = distance.cdist(self.currPos, self.waypoints, 'euclidean').reshape((self.num_waypoints))
        self.closest_index = np.argmin(self.distances)
        self.closestPoint = self.waypoints[self.closest_index]

        # Find target point
        targetPoint = self.get_closest_point_beyond_lookahead_dist(self.L)

        # Homogeneous transformation
        translatedTargetPoint = self.translatePoint(targetPoint)

        pose_arr_msg = PoseArray()
        pose_msg_target = Pose()
        pose_msg_target.position.x = translatedTargetPoint[0]
        pose_msg_target.position.y = translatedTargetPoint[1]
        pose_arr_msg.poses.append(pose_msg_target)
        pose_msg_closest = Pose()
        pose_msg_closest.position.x = self.closestPoint[0]
        pose_msg_closest.position.y = self.closestPoint[1]
        pose_arr_msg.poses.append(pose_msg_closest)
        pose_msg_curr = Pose()
        pose_msg_curr.position.x = self.currX
        pose_msg_curr.position.y = self.currY
        pose_arr_msg.poses.append(pose_msg_curr)
        
        self.pub_to_gf.publish(pose_arr_msg)

        # calculate curvature/steering angle
        # y = translatedTargetPoint[1]
        # gamma = self.steering_gain * (2 * y / self.L**2)

        # # publish drive message, don't forget to limit the steering angle.
        # gamma = np.clip(gamma, -0.35, 0.35)
        # self.drive_msg.drive.steering_angle = gamma
        # self.drive_msg.drive.speed = (-1.0 if self.is_real else 1.0) * self.ref_speed[self.closest_index]
        # self.pub_drive.publish(self.drive_msg)
        # print("steering = {}, speed = {}".format(round(self.drive_msg.drive.steering_angle, 2), round(self.drive_msg.drive.speed, 2)))

        # Visualizing points
        self.closestMarker.points = [Point(x = self.closestPoint[0], y = self.closestPoint[1], z = 0.0)]
        self.targetMarker.points = [Point(x = targetPoint[0], y = targetPoint[1], z = 0.0)]

        self.markerArray.markers = [self.waypointMarker, self.closestMarker, self.targetMarker]
        self.pub_vis.publish(self.markerArray)
        
    def gap_following_callback(self, gf_point_msg):
        gf_point_x = gf_point_msg.x
        gf_point_y = gf_point_msg.y
        gf_point_obs = bool(gf_point_msg.z)
        # print(f'received gf point:',gf_point_msg.x, gf_point_msg.y)

        self.gf_point_marker.points = [Point(x = gf_point_x, y = gf_point_y, z = 0.2)]
        self.vis_gf_point_pub.publish(self.gf_point_marker)

        # calculate speed & steering
        gamma = self.steering_gain * (2 * gf_point_y / self.L**2)
        gamma = np.clip(gamma, -0.35, 0.35)
        self.drive_msg.drive.steering_angle = gamma
        speed = (-1.0 if self.is_real else 1.0) * self.ref_speed[self.closest_index]
        # self.drive_msg.drive.speed = (0.2 if gf_point_obs else 1.0) * speed  # if allows braking for close obstacles
        self.drive_msg.drive.speed = speed
        # self.drive_msg.drive.speed = self.test_speed
        
        # publish drive message
        self.pub_drive.publish(self.drive_msg)
        print("steering = {}, speed = {}".format(round(self.drive_msg.drive.steering_angle, 2), round(self.drive_msg.drive.speed, 2)))

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
    
    def visualization_init(self):
        # Green
        self.waypointMarker = Marker()
        self.waypointMarker.header.frame_id = 'map'
        self.waypointMarker.type = Marker.POINTS
        self.waypointMarker.color.g = 0.75
        self.waypointMarker.color.a = 1.0
        self.waypointMarker.scale.x = 0.05
        self.waypointMarker.scale.y = 0.05
        self.waypointMarker.id = 0
        self.waypointMarker.points = [Point(x = wpt[0], y = wpt[1], z = 0.0) for wpt in self.waypoints]

        # Red
        self.targetMarker = Marker()
        self.targetMarker.header.frame_id = 'map'
        self.targetMarker.type = Marker.POINTS
        self.targetMarker.color.r = 0.75
        self.targetMarker.color.a = 1.0
        self.targetMarker.scale.x = 0.2
        self.targetMarker.scale.y = 0.2
        self.targetMarker.id = 1

        # Blue
        self.closestMarker = Marker()
        self.closestMarker.header.frame_id = 'map'
        self.closestMarker.type = Marker.SPHERE_LIST
        self.closestMarker.color.b = 0.75
        self.closestMarker.color.a = 0.4
        self.closestMarker.scale.x = 1.2
        self.closestMarker.scale.y = 1.2
        self.closestMarker.scale.z = 1.2
        self.closestMarker.id = 2

        # green
        self.gf_point_marker = Marker()
        self.gf_point_marker.header.frame_id = 'laser' if self.is_real else 'ego_racecar/base_link'
        self.gf_point_marker.type = Marker.POINTS
        self.gf_point_marker.color.g = 0.75
        self.gf_point_marker.color.a = 1.0
        self.gf_point_marker.scale.x = 0.2
        self.gf_point_marker.scale.y = 0.2
        self.gf_point_marker.id = 3


def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
