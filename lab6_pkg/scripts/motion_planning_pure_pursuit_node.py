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


class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.is_real = False

        # Topics & Subs, Pubs
        drive_topic = '/drive'
        odom_topic = '/pf/viz/inferred_pose' if self.is_real else '/ego_racecar/odom'
        visualization_topic = '/visualization_marker_array'
        pp_goal_topic = '/pp_goal'
        rrt_goal_topic = '/rrt_goal'

        # Subscribe to POSE
        self.sub_pose = self.create_subscription(PoseStamped if self.is_real else Odometry, odom_topic, self.pose_callback, 1)
        self.sub_pose
        
        # Publish to drive
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.drive_msg = AckermannDriveStamped()

        # Publish to visualization
        self.pub_vis = self.create_publisher(MarkerArray, visualization_topic, 1)

        # FOR RRT LAB
        # Publish goal point in car frame to RRT node
        self.pub_to_rrt = self.create_publisher(Point, pp_goal_topic, 1)
        # Subscribe to RRT points
        self.sub_rrt = self.create_subscription(Point, rrt_goal_topic, self.rrt_callback, 1)
        self.sub_rrt

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
        
        # Purple/pink
        self.ppGoalPoint = Marker()
        self.ppGoalPoint.header.frame_id = 'ego_racecar/base_link'
        self.ppGoalPoint.type = Marker.POINTS
        self.ppGoalPoint.color.b = 1.0
        self.ppGoalPoint.color.r = 1.0
        self.ppGoalPoint.color.a = 1.0
        self.ppGoalPoint.scale.x = 0.2
        self.ppGoalPoint.scale.y = 0.2
        self.ppGoalPoint.id = 3

        # Black
        self.rrtGoalPoint = Marker()
        # self.rrtGoalPoint.header.frame_id = 'map'
        self.rrtGoalPoint.header.frame_id = 'ego_racecar/base_link'
        self.rrtGoalPoint.type = Marker.POINTS
        self.rrtGoalPoint.color.b = 0.0
        self.rrtGoalPoint.color.r = 0.0
        self.rrtGoalPoint.color.g = 0.0
        self.rrtGoalPoint.color.a = 1.0
        self.rrtGoalPoint.scale.x = 0.2
        self.rrtGoalPoint.scale.y = 0.2
        self.rrtGoalPoint.id = 4

        self.waypoints = np.asarray(generate_waypoints()).T
        self.waypointMarker.points = [Point(x = x, y = y, z = 0.0) for x, y in self.waypoints]
        self.numWaypoints = self.waypoints.shape[0]

        self.markerArray = MarkerArray()
        
        self.L = 3.0
        self.steeringGain = 2.0
        self.test_speed = 0.6

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

        # Find waypoint L away or just beyond
        nearTargetPoint = self.getNearTargetPoint(self.rot, self.L, 2.5 * self.L)
        # nearTargetPoint = self.get_closest_point_beyond_lookahead_dist()

        # Find target point
        # targetPoint = self.findTargetPoint(self.currPos, self.closestPoint, nearTargetPoint)
        targetPoint = nearTargetPoint

        # Homogeneous transformation
        translatedTargetPoint = self.translatePoint(targetPoint)
        self.pub_to_rrt.publish(Point(x = translatedTargetPoint[0], y = translatedTargetPoint[1], z = 0.0))

        # original pp code to drive!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        # # calculate curvature/steering angle
        # y = translatedTargetPoint[1]
        # gamma = self.steeringGain * (2 * y / self.L**2)

        # # publish drive message, don't forget to limit the steering angle.
        # gamma = np.clip(gamma, -0.35, 0.35)
        # self.drive_msg.drive.steering_angle = gamma
        # print('steering = ',round(self.drive_msg.drive.steering_angle, 2))
        # # self.drive_msg.drive.speed = self.velocityLookupTable()
        # self.drive_msg.drive.speed = (-1.0 if self.is_real else 1.0) * self.getVelocity() * 0.1
        # print('speed = ', round(self.drive_msg.drive.speed, 2))
        # self.pub_drive.publish(self.drive_msg)

        # Visualizing points
        # self.markerArray = MarkerArray()
        self.targetMarker.points = [Point(x = targetPoint[0], y = targetPoint[1], z = 0.0)]
        self.closestMarker.points = [Point(x = self.closestPoint[0], y = self.closestPoint[1], z = 0.0)]
        self.markerArray.markers = [self.waypointMarker, self.targetMarker, self.closestMarker]

        # purple is same as red?
        # self.ppGoalPoint.points = [Point(x = translatedTargetPoint[0], y = translatedTargetPoint[1], z = 0.0)]
        # self.markerArray.markers.append(self.ppGoalPoint)

        # publisher moved to rrt_callback so that it can print out  the rrt goal point
        # self.markerArray.markers.append(self.rrtGoalPoint)
        # self.pub_vis.publish(self.markerArray)

    def rrt_callback(self, rrt_goal_msg):
        rrt_x = rrt_goal_msg.x
        rrt_y = rrt_goal_msg.y
        # print(f'received rrt goal message:',rrt_goal_msg.x, rrt_goal_msg.y)

        # rrt_target = np.array((rrt_x, rrt_y)).reshape((2))
        # rrt_target = self.translatePoint(rrt_target)
        # print(rrt_target)
        # print(rrt_target[0], rrt_target[1])
        ## Commented = input is car frame, uncommented = input is world frame
        # rrt_x = rrt_target[0]
        # rrt_y = rrt_target[1]

        self.rrtGoalPoint.points = [Point(x = rrt_x, y = rrt_y, z = 0.2)]
        # self.rrtGoalPoint.points = [Point(x = rrt_target[0], y = rrt_target[1], z = 0.2)]
        self.markerArray.markers.append(self.rrtGoalPoint)

        self.pub_vis.publish(self.markerArray)

        # calculate curvature/steering angle
        gamma = self.steeringGain * (2 * rrt_y / self.L**2)

        # publish drive message, don't forget to limit the steering angle.
        gamma = np.clip(gamma, -0.35, 0.35)
        self.drive_msg.drive.steering_angle = gamma
        # self.drive_msg.drive.speed = (-1.0 if self.is_real else 1.0) * self.getVelocity()
        self.drive_msg.drive.speed = self.test_speed
        print("steering = ", gamma)
        self.pub_drive.publish(self.drive_msg)

    def getNearTargetPoint(self, rot, lowerL, upperL):

        nearIndices = np.asarray((self.distances >= lowerL) & (self.distances < upperL)).nonzero()[0]
        currNearDistances = self.distances[nearIndices]

        nextPos = self.currPos + 0.01 * self.velocityLookupTable() * rot[0:2, 0]

        nextNearDistances = distance.cdist(nextPos, self.waypoints[nearIndices], 'euclidean').reshape((len(nearIndices)))

        gettingCloserIndices = np.asarray(nextNearDistances < currNearDistances).nonzero()[0]
        if len(nextNearDistances[gettingCloserIndices]) != 0:
            gettingCloserClosestIndex = gettingCloserIndices[nextNearDistances[gettingCloserIndices].argmin()]
            closestWaypointIndex = nearIndices[gettingCloserClosestIndex]
            nearTargetPoint = self.waypoints[closestWaypointIndex]
            return nearTargetPoint
        else:
            nearTargetPoint = self.closestPoint
            print("nearTargetPoint is closestPoint!")
            return nearTargetPoint

    def velocityLookupTable(self):
        #TODO: Implement this
        turning = 3.5
        straight = 5.0

        upperX = 6
        lowerX = -10.0
        rightY = 2
        leftY = 5.5

        if self.currX > upperX and self.currY < rightY:
            return turning
        elif self.currX > upperX and self.currY > leftY:
            return turning
        elif self.currX < lowerX and self.currY > leftY:
            return turning
        elif self.currX < lowerX and self.currY < rightY:
            return turning
        else:
            return straight
        
    # I'm not a big fan of this implementation bc you have to manually input which way you're going, which is fine but somewhat tedious
    def get_closest_point_beyond_lookahead_dist(self):
        
        point_index = np.argmin(self.distances)
        dist = self.distances[point_index]
        while dist < self.L:
            # point_index += 1
            # if point_index >= len(self.waypoints):
            #     point_index = 0
            point_index -= 1
            if point_index < 0:
                point_index = len(self.waypoints) - 1
            dist = self.distances[point_index]
        
        point = self.waypoints[point_index]

        return point

    # def getVelocity(self):
    #     minV = 4.0
    #     nominalV = 8.0
    #     lowerMultiplier = 4.0
    #     upperMultiplier = 5.5
    #     gain = 1.25

    #     lowerL = lowerMultiplier * self.L
    #     upperL = upperMultiplier * self.L
    #     speedTarget = self.getNearTargetPoint(self.rot, lowerL, upperL)
    #     # speedTarget = self.get_closest_point_beyond_lookahead_dist(self.distances)
    #     translatedSpeedTarget = self.translatePoint(speedTarget)

    #     theta = np.arctan2(translatedSpeedTarget[1], translatedSpeedTarget[0])
    #     theta = np.abs(theta)
    #     scaler = 1 - (gain * theta / (np.pi / 2.0))
    #     velocity = nominalV * scaler
        
    #     return np.clip(velocity, minV, nominalV)

    def translatePoint(self, targetPoint):
        H = np.zeros((4, 4))
        H[0:3, 0:3] = np.linalg.inv(self.rot)
        H[0, 3] = self.currX
        H[1, 3] = self.currY
        H[3, 3] = 1.0
        pvect = targetPoint - self.currPos
        convertedTarget = (H @ np.array((pvect[0, 0], pvect[0, 1], 0, 0))).reshape((4))
        
        return convertedTarget

def generate_waypoints():
    # sampled from slam maps directly
    points_x = [8.5296, 7.13217, -1.57949, -11.5957, -13.0057, 
                -13.7761, -13.7792, -13.8221, 
                -13.0357, -11.6245, -2.26039, 7.15083, 8.54562, 
                9.29226, 9.2999, 9.31468, 
                8.5296]
    points_y = [-0.27739, -0.322446, -0.337913, -0.355279, -0.360477, 
                0.511654, 4.1081, 7.79924, 
                8.48277, 8.54935, 8.4717, 8.43548,  8.38767, 
                7.66307, 4.0313, 0.468403, 
                -0.27739]

    tck, u = splprep([points_x, points_y], s=0, per=True)
    new_points = splev(np.linspace(0, 1, 500), tck)

    return new_points


def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
