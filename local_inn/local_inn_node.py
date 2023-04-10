#!/usr/bin/env python3

from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, PoseArray
from tf2_ros import TransformBroadcaster
import tf_transformations
import numpy as np
import importlib
import time
from nav_msgs.msg import Odometry
import utils as Utils
from EKF import EKF
from scipy.stats import circvar
import matplotlib.pyplot as plt
from local_inn_trt import Local_INN_TRT_Runtime

PUBLISH_LASER_FRAME = 1
PUBLISH_ODOM = 0
USE_EFK = 1
USE_TRT = 1
EKF_parameters = [100, 100] # levine1 1m/s 36
JUMP_THRESHOLD = 1 # large jump will be filtered

class Local_INN(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('local_inn_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        used_lidarscan_topic = '/local_inn/used_scan'

        self.scan_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 1)
        self.particle_pub = self.create_publisher(PoseArray, '/local_inn/particles', 1)
        self.pose_publisher = self.create_publisher(PoseStamped, '/local_inn/pose', 1)
        self.odom_pub = self.create_publisher(Odometry, '/local_inn/odom', 1)
        self.used_scan_pub = self.create_publisher(LaserScan, used_lidarscan_topic, 1)
        
        EXP_NAME = 'inn_exp36' # levine1
        self.local_inn = Local_INN_TRT_Runtime(EXP_NAME, USE_TRT, 20)
        self.hz_cnt = 0
        self.hz = 0

        self.laser_br = TransformBroadcaster(self)
        
        
        self.initialpose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 1)
        self.prev_state = np.zeros(3)
        self.prev_scan = None # for solve bad point problem
        
        self.initialpose_flag = 0

        # self.odom_sub = self.create_subscription(Odometry,
        #                                          '/ego_racecar/odom',
        #                                          self.odom_callback, 1)
        self.odom_sub = self.create_subscription(Odometry,
                                                 '/odom',
                                                 self.odom_callback, 1)
        self.odometry_data = np.array([0.0, 0.0, 0.0])
        self.pre_odom = None
        
        self.ekf = EKF()
        self.xEst = np.zeros((4, 1))  # estimated state vector
        self.xEst[:3, 0] = self.prev_state.copy()
        self.PEst = np.eye(4)  # estimated covariance matrix
        self.xPred = self.xEst.copy()
        self.PPred = self.PEst.copy()
        self.ekf_update_flag = 0
        self.record = []
        
        self.speed_scan = 0
        self.speed_odom = 0
        
    def two_pi_warp(self, angles):
        twp_pi = 2 * np.pi
        return (angles + twp_pi) % (twp_pi)

        
    def initialpose_callback(self, msg):
        state = np.zeros(3)
        state[0] = msg.pose.pose.position.x
        state[1] = msg.pose.pose.position.y
        state[2] = Rotation.from_quat([msg.pose.pose.orientation.x,
                                  msg.pose.pose.orientation.y,
                                  msg.pose.pose.orientation.z,
                                  msg.pose.pose.orientation.w]).as_euler('zxy', degrees=False)[0]
        
        if state[2] < 0:
            state[2] = state[2] + np.pi * 2
        self.prev_state = state.copy()
        self.xEst[:3, 0] = self.prev_state.copy()
        self.xPred = self.xEst.copy()
        print('initialpose', state)
        

    def scan_callback(self, msg):
        ## removing bad points in lidar
        scan = np.array(msg.ranges)[np.arange(0, 1080, 4)]
        error_points = np.where((scan == 0.001))[0]
        for ind in error_points:
            ind_search = ind
            while ind_search >= 0 and ind_search < 270 and scan[ind_search] - 0.001 < 1e-5:
                ind_search += 1
                if ind_search == 270:
                    ind_search = 0
            scan[ind] = scan[ind_search]

        start_time = time.time()
        inferred_state, inferred_states = self.local_inn.reverse(scan, self.prev_state.copy())

        if np.linalg.norm(inferred_state[:2] - self.prev_state[:2]) < JUMP_THRESHOLD:
            if not USE_EFK:
                self.prev_state = inferred_state.copy()
            else:
                inferred_states[np.where(inferred_states[:, 2] < 0)][:, 2] += np.pi * 2
                self.two_pi_warp(inferred_states[:, 2])
                covariance = np.cov(inferred_states.T) * EKF_parameters[1]
                covariance[:2, :2] *= EKF_parameters[0]
                
                self.xEst, self.PEst, K = self.ekf.update(self.xPred, self.PPred, inferred_state[:, None], covariance)
                self.ekf_update_flag = 1
                if self.xEst[2, 0] < 0: self.xEst[2, 0] += np.pi * 2
                self.two_pi_warp(self.xEst[2, 0])
                self.prev_state = self.xEst[:3, 0].copy()


        self.hz += 1/(time.time() - start_time)
        self.hz_cnt += 1
        if self.hz_cnt == 100:
            print(self.hz/100, 'Hz')
            self.hz_cnt = 0
            self.hz = 0

        # publish inferred pose
        if self.pose_publisher.get_subscription_count() > 0:
            new_pose = PoseStamped()
            new_pose.header.stamp = msg.header.stamp
            new_pose.header.frame_id = '/map'
            new_pose.pose.position.x = self.prev_state[0]
            new_pose.pose.position.y = self.prev_state[1]
            yaw = Rotation.from_euler('z', self.prev_state[2], degrees=False).as_quat()
            new_pose.pose.orientation.x = yaw[0]
            new_pose.pose.orientation.y = yaw[1]
            new_pose.pose.orientation.z = yaw[2]
            new_pose.pose.orientation.w = yaw[3]
            self.pose_publisher.publish(new_pose)
        
        # publish inferred state samples
        if self.particle_pub.get_subscription_count() > 0:
            self.publish_particles(inferred_states, msg.header.stamp)

        if PUBLISH_LASER_FRAME:
            laser_t = TransformStamped()
            laser_t.header.stamp = msg.header.stamp
            laser_t.header.frame_id = 'map'
            laser_t.child_frame_id = 'laser_local_inn'

            laser_t.transform.translation.x = self.prev_state[0]
            laser_t.transform.translation.y = self.prev_state[1]
            laser_t.transform.translation.z = 0.0

            q = tf_transformations.quaternion_from_euler(0, 0, self.prev_state[2])
            laser_t.transform.rotation.x = q[0]
            laser_t.transform.rotation.y = q[1]
            laser_t.transform.rotation.z = q[2]
            laser_t.transform.rotation.w = q[3]

            # Send the transformation
            self.laser_br.sendTransform(laser_t)

        if PUBLISH_ODOM:
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = '/map'
            odom.pose.pose.position.x = self.prev_state[0]
            odom.pose.pose.position.y = self.prev_state[1]
            odom.pose.pose.orientation = Utils.angle_to_quaternion(self.prev_state[2])
            if isinstance(self.pre_odom, np.ndarray): 
                odom.twist.twist.linear.x = self.pre_odom[0]
                odom.twist.twist.angular.z = self.pre_odom[1]
            self.odom_pub.publish(odom)
        
        if self.used_scan_pub.get_subscription_count() > 0:
            used_scan_msg = msg
            used_scan_msg.header.frame_id = 'laser_local_inn'
            used_scan_msg.angle_increment = msg.angle_increment * 4
            used_scan_msg.angle_min = msg.angle_min
            used_scan_msg.angle_max = msg.angle_max
            used_scan_msg.ranges = list(scan.data)
            self.used_scan_pub.publish(used_scan_msg)
        
    def odom_callback(self, msg):
        odom_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        linear_speed = msg.twist.twist.linear.x
        angular_speed = msg.twist.twist.angular.z
        
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y])
        orientation = Utils.quaternion_to_angle(msg.pose.pose.orientation)
        pose = np.array([position[0], position[1], orientation])
        
        if isinstance(self.pre_odom, np.ndarray):            
            delta_time = odom_time - self.pre_odom[2]
            if not USE_EFK:
                delta_pose = np.array([linear_speed * np.cos(self.prev_state[2]),
                                    linear_speed * np.sin(self.prev_state[2]),
                                    angular_speed]) * delta_time
                self.prev_state += delta_pose
                # pass
            else:
                rot = Utils.rotation_matrix(-self.last_pose[2])
                delta = np.array([position - self.last_pose[0:2]]).transpose()
                local_delta = (rot*delta).transpose()
                self.odometry_data = np.array([local_delta[0,0], local_delta[0,1], orientation - self.last_pose[2]])
                self.last_pose = pose
                
                u = np.array([[np.sqrt(self.odometry_data[0] ** 2 + self.odometry_data[1] ** 2) / delta_time], 
                                [self.odometry_data[2] / delta_time]])
                # u = np.array([[linear_speed], [angular_speed]])
                if self.ekf_update_flag == 1:
                    self.xPred, self.PPred = self.ekf.predict(self.xEst.copy(), self.PEst.copy(), u, delta_time)
                else:
                    self.xPred, self.PPred = self.ekf.predict(self.xPred.copy(), self.PPred.copy(), u, delta_time)
                self.ekf_update_flag = 0
                if self.xPred[2, 0] < 0: self.xPred[2, 0] += np.pi * 2
                self.two_pi_warp(self.xPred[2, 0])
                self.prev_state = self.xPred[:3, 0].copy()
                
            self.pre_odom = np.array([linear_speed, angular_speed, odom_time])
        else:
            self.last_pose = pose
            self.pre_odom = np.array([linear_speed, angular_speed, odom_time])
            
    def publish_particles(self, particles, time_stamp):
        # publish the given particles as a PoseArray object
        pa = PoseArray()
        pa.header.stamp = time_stamp
        pa.header.frame_id = '/map'
        pa.poses = Utils.particles_to_poses(particles)
        self.particle_pub.publish(pa)


def main(args=None):
    rclpy.init(args=args)
    print("Local_INN Initializing.")
    local_inn_node = Local_INN()
    rclpy.spin(local_inn_node)

    local_inn_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
