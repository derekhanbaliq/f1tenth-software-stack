#!/usr/bin/env python3

"""
    MEAM 517 Final Project - LQR Steering Speed Control - LQR class
    Author: Derek Zhou & Tancy Zhao
    References: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/lqr_steer_control
                https://github.com/f1tenth/f1tenth_planning/tree/main/f1tenth_planning/control/lqr
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

from utils import calc_nearest_point, pi_2_pi


class Waypoint:
    def __init__(self, map_name, is_real=False, is_ascending=True, csv_data=None):
        self.x = csv_data[:, 1]
        self.y = csv_data[:, 2]
        if is_real:
            self.v = csv_data[:, 5] * 0.5  # use 0 ~ 5 m/s speed for real condition
        else:
            self.v = csv_data[:, 5]
        if map_name == 'levine_2nd':
            self.θ = csv_data[:, 3] + (math.pi / 2 if is_ascending else - math.pi / 2)  # coordinate matters!
        else:
            self.θ = csv_data[:, 3]
        self.γ = csv_data[:, 4]


class CarState:
    def __init__(self, x=0.0, y=0.0, θ=0.0, v=0.0):
        self.x = x
        self.y = y
        self.θ = θ
        self.v = v


class LKVMState:
    """
    Linear Kinematic Vehicle Model's state space expression
    """
    def __init__(self, e_l=0.0, e_l_dot=0.0, e_θ=0.0, e_θ_dot=0.0, v=0.0):
        # 4 states
        self.e_l = e_l
        self.e_l_dot = e_l_dot
        self.e_θ = e_θ
        self.e_θ_dot = e_θ_dot
        self.e_v = v
        # log old states
        self.old_e_l = 0.0
        self.old_e_θ = 0.0

    def update(self, e_l, e_θ, e_v, dt):  # pack-up rather than calc
        self.e_l = e_l
        self.e_l_dot = (e_l - self.old_e_l) / dt
        self.e_θ = e_θ
        self.e_θ_dot = (e_θ - self.old_e_θ) / dt
        self.e_v = e_v  # e_l, e_θ, e_v have been calculated previously

        x = np.vstack([self.e_l, self.e_l_dot, self.e_θ, self.e_θ_dot, self.e_v])

        return x


class LQRSolver:
    def __init__(self, dt, l_wb, v=0.0):
        self.A = np.array([[1.0,    dt,     0,       0,         0],
                           [0,      0,      v,       0,         0],
                           [0,      0,      1.0,     dt,        0],
                           [0,      0,      0,       0,         0],
                           [0,      0,      0,       0,         1.0]])
        self.B = np.array([[0,          0],
                           [0,          0],
                           [0,          0],
                           [v / l_wb,   0],
                           [0,          dt]])  # l_wb is wheelbase
        self.Q = np.diag([1.5, 0.5, 1.0, 0.5, 0.5])
        self.R = np.diag([1, 1])

    def discrete_lqr(self):
        A = self.A
        B = self.B
        R = self.R

        S = self.solve_recatti_equation()
        K = -np.linalg.pinv(B.T @ S @ B + R) @ (B.T @ S @ A)  # u = -(B.T @ S @ B + R)^(-1) @ (B.T @ S @ A) @ x[k]

        return K  # K is 2 x 5

    def solve_recatti_equation(self):
        A = self.A
        B = self.B
        Q = self.Q
        R = self.R  # just for simplifying the following recatti expression

        S = self.Q
        Sn = None

        max_iter = 100
        ε = 0.001  # tolerance epsilon
        diff = math.inf  # always use value iteration with max iteration!

        # print('S0 = Q = {}'.format(self.Q))

        i = 0
        while i < max_iter and diff > ε:
            i += 1
            Sn = Q + A.T @ S @ A - (A.T @ S @ B) @ np.linalg.pinv(R + B.T @ S @ B) @ (B.T @ S @ A)
            S = Sn

        # print('Sn = {}'.format(Sn))

        return Sn


class LQR(Node):
    def __init__(self):
        super().__init__('lqr_node')

        self.is_real = False
        self.is_ascending = True  # waypoint indices are ascending during tracking
        self.map_name = 'levine_2nd'

        # Topics & Subs, Pubs
        drive_topic = '/drive'
        odom_topic = '/pf/viz/inferred_pose' if self.is_real else '/ego_racecar/odom'
        visualization_topic = '/visualization_marker_array'
        pf_vel_topic = '/pf/pose/odom'

        # Subscribe to pose
        self.sub_pose = self.create_subscription(PoseStamped if self.is_real else Odometry, odom_topic, self.pose_callback, 1)
        # Subscribe to pf odom for current speed
        if self.is_real:
            self.sub_vel = self.create_subscription(Odometry, pf_vel_topic, self.pf_vel_callback, 1)
        # Publish to drive
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.drive_msg = AckermannDriveStamped()
        # Publish to visualization
        self.pub_vis = self.create_publisher(MarkerArray, visualization_topic, 1)
        self.markerArray = MarkerArray()

        # load waypoints
        map_path = os.path.abspath(os.path.join('src', 'csv_data'))
        csv_data = np.loadtxt(map_path + '/' + self.map_name + '.csv', delimiter=';', skiprows=0)  # csv data
        self.waypoints = Waypoint(self.map_name, self.is_real, self.is_ascending, csv_data)

        self.visualization_init()

        self.dt = 0.01  # time step
        self.wheelbase = 0.33
        self.car = CarState()
        self.x = LKVMState()
        self.pf_curr_v = 0.0  # only for real car

    def pose_callback(self, pose_msg):
        # get current pose
        self.curr_x = pose_msg.pose.position.x if self.is_real else pose_msg.pose.pose.position.x
        self.curr_y = pose_msg.pose.position.y if self.is_real else pose_msg.pose.pose.position.y
        self.currPos = np.array([self.curr_x, self.curr_y]).reshape((1, 2))

        # transform quaternion pose message to rotation matrix
        quat = pose_msg.pose.orientation if self.is_real else pose_msg.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        R = transform.Rotation.from_quat(q)
        self.rot = R.as_matrix()

        # get current speed & yaw
        curr_v = self.pf_curr_v if self.is_real else pose_msg.twist.twist.linear.x
        curr_yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))

        steering, speed = self.lqr_steering_speed_control(self.curr_x, self.curr_y, curr_v, curr_yaw)
        print("steering = {}, speed = {}".format(round(steering, 5), round(speed, 5)))

        # publish drive message
        self.drive_msg.drive.steering_angle = steering
        self.drive_msg.drive.speed = (-1.0 if self.is_real else 1.0) * speed
        self.pub_drive.publish(self.drive_msg)

        self.markerArray.markers = [self.waypointMarker, self.front_marker]
        self.pub_vis.publish(self.markerArray)
        
    def pf_vel_callback(self, odom_msg):
        # get current speed from the odom msg of particle filter
        self.pf_curr_v = odom_msg.twist.twist.linear.x
        print('current velocity via pf = ', round(self.pf_curr_v, 5))

    def lqr_steering_speed_control(self, curr_x, curr_y, curr_v, curr_yaw):
        """
        LQR steering speed control for Lateral Kinematics Vehicle Model
        """
        self.car.x = curr_x
        self.car.y = curr_y
        self.car.θ = curr_yaw
        self.car.v = curr_v  # each agent’s current longitudinal velocity

        self.x.old_e_l = self.x.e_l
        self.x.old_e_θ = self.x.e_θ  # log into x's static variables

        e_l, e_θ, γ, e_v = self.calc_control_points()  # Calculate errors and reference point

        lqr = LQRSolver(self.dt, self.wheelbase, self.car.v)  # init A B Q R with the current car state
        K = lqr.discrete_lqr()  # use A, B, Q, R to get K

        x_new = self.x.update(e_l, e_θ, e_v, self.dt)  # x[k+1]

        steering_fb = (K @ x_new)[0, 0]  # K is 2 x 5, x is 5 x 1
        # feedforward_term = math.atan2(self.wheelbase * γ, 1)  # = math.atan2(L / r, 1) = math.atan2(L, r)
        steering_ff = self.wheelbase * γ  # can be seen as current steering angle

        speed_fb = (K @ x_new)[1, 0]  # the acceleration, should be regarded as Δv, or acceleration in 0.01s

        steering = - steering_fb + steering_ff
        speed = - speed_fb + self.car.v  # current car speed is the base, v_base + Δv is the speed we want
        # speed = - accel * self.dt + self.car.v is wrong because we are in the loop, Δt should be "unit 1"

        return steering, speed

    def get_front_pos(self):
        front_x = self.car.x + self.wheelbase * math.cos(self.car.θ)
        front_y = self.car.y + self.wheelbase * math.sin(self.car.θ)
        front_pos = np.array([front_x, front_y])

        self.front_marker.points = [Point(x = front_x, y = front_y, z = 0.25)]

        return front_pos

    def calc_control_points(self):
        front_pos = self.get_front_pos()

        waypoint_i, min_d, _, i = \
            calc_nearest_point(front_pos, np.array([self.waypoints.x, self.waypoints.y]).T)

        waypoint_to_front = front_pos - waypoint_i  # regard this as a vector

        front_axle_vec_rot_90 = np.array([[math.cos(self.car.θ - math.pi / 2.0)],
                                          [math.sin(self.car.θ - math.pi / 2.0)]])
        e_l = np.dot(waypoint_to_front.T, front_axle_vec_rot_90)  # real lateral error, the horizontal dist

        # e_θ = (self.waypoints.θ[i] - self.car.θ)
        e_θ = pi_2_pi(self.waypoints.θ[i] - self.car.θ) # heading error
        # print('e_l = ', e_l)
        # print('e_θ = ', e_θ)

        γ = self.waypoints.γ[i]  # curvature of the nearst waypoint

        e_v = self.waypoints.v[i] - self.car.v  # velocity of the nearst waypoint

        return e_l, e_θ, γ, e_v

    def translatePoint(self, targetPoint):
        H = np.zeros((4, 4))
        H[0:3, 0:3] = np.linalg.inv(self.rot)
        H[0, 3] = self.curr_x
        H[1, 3] = self.curr_y
        H[3, 3] = 1.0
        pvect = targetPoint - self.currPos
        convertedTarget = (H @ np.array((pvect[0, 0], pvect[0, 1], 0, 0))).reshape((4))
        
        return convertedTarget
    
    def visualization_init(self):
        # Green
        self.waypointMarker = Marker()
        self.waypointMarker.header.frame_id = 'map'
        self.waypointMarker.type = Marker.POINTS
        self.waypointMarker.color.r = 0.75
        self.waypointMarker.color.a = 1.0
        self.waypointMarker.scale.x = 0.05
        self.waypointMarker.scale.y = 0.05
        self.waypointMarker.id = 0
        self.waypointMarker.points = [Point(x = self.waypoints.x[i], y = self.waypoints.y[i], z = 0.0) for i in range(len(self.waypoints.x))]
        
        # Cyan
        self.front_marker = Marker()
        self.front_marker.header.frame_id = 'map'
        self.front_marker.type = Marker.POINTS
        self.front_marker.color.g = 1.0
        self.front_marker.color.b = 1.0
        self.front_marker.color.a = 1.0
        self.front_marker.scale.x = 0.2
        self.front_marker.scale.y = 0.2
        self.front_marker.id = 2

def main(args=None):
    rclpy.init(args=args)
    print("Lateral Kinematic LQR Initialized")
    lqr_node = LQR()
    rclpy.spin(lqr_node)

    lqr_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

