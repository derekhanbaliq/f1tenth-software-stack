#!/usr/bin/env python3

from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
import numpy as np
import time
from nav_msgs.msg import Odometry


class TrajectoryRecord(Node):

    def __init__(self):
        super().__init__('trajectory_record_node')

        self.trajectory_record = {}
        self.trajectory_record['time_stamp'] = []
        self.trajectory_record['state'] = []
        
        self.scan_record = {}
        self.scan_record['time_stamp'] = []
        self.scan_record['scan'] = []
        
        self.odom_record = {}
        self.odom_record['time_stamp'] = []
        self.odom_record['odom'] = []
        self.odom_record['delta_time'] = []
        
        self.particle_filter_sub = self.create_subscription(PoseStamped,
                                                            '/pf/viz/inferred_pose',
                                                            self.particle_filter_callback, 1)

        self.initialpose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                        '/initialpose',
                                                        self.initialpose_callback, 1)

        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)

        self.local_inn_sub = self.create_subscription(PoseStamped,
                                                      '/local_inn/pose',
                                                      self.local_inn_callback, 1)

        lidarscan_topic = '/scan'
        lidarscan_topic = '/scan'
        self.scan_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 1)
        
        self.odom_sub = self.create_subscription(Odometry,
                                                 '/odom',
                                                 self.odom_callback, 1)
        self.pre_odom = None

        msg = PoseWithCovarianceStamped()
        initial_state = np.array([-5.39598927, 0.02058008, 0.08275554]) # levine_1
        # initial_state = np.array([2.80070045, 8.44312642, 2.86860034]) # outdoor1
        # initial_state = np.array([1.84822595, 9.11884468, 3.24810417]) # outdoor11
        # initial_state = np.array([3.17877412, 8.45598508, 2.86190808])  # outdoor10

        msg.pose.pose.position.x = initial_state[0]
        msg.pose.pose.position.y = initial_state[1]
        quat = Rotation.from_euler('z', initial_state[2], degrees=False).as_quat()
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        msg.header.frame_id = 'map'
        self.initialpose_pub.publish(msg)

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

        time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        print('initialpose', state)

    def particle_filter_callback(self, msg):
        state = np.zeros(3)
        state[0] = msg.pose.position.x
        state[1] = msg.pose.position.y
        state[2] = Rotation.from_quat([msg.pose.orientation.x,
                                       msg.pose.orientation.y,
                                       msg.pose.orientation.z,
                                       msg.pose.orientation.w]).as_euler('zxy', degrees=False)[0]
        if state[2] < 0:
            state[2] = state[2] + np.pi * 2
        print('particle_filter', state)

        time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        self.trajectory_record['time_stamp'].append(time_stamp)
        self.trajectory_record['state'].append(state)

    def local_inn_callback(self, msg):
        state = np.zeros(3)
        state[0] = msg.pose.position.x
        state[1] = msg.pose.position.y
        state[2] = Rotation.from_quat([msg.pose.orientation.x,
                                       msg.pose.orientation.y,
                                       msg.pose.orientation.z,
                                       msg.pose.orientation.w]).as_euler('zxy', degrees=False)[0]
        if state[2] < 0:
            state[2] = state[2] + np.pi * 2

        time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        self.trajectory_record['time_stamp'].append(time_stamp)
        self.trajectory_record['state'].append(state)

    def scan_callback(self, msg):
        # self.get_logger().info(f"Scan Callback: {msg}")
        scan = np.array(msg.ranges)[np.arange(0, 1080, 4)]
        error_points = np.where(scan == 0.001)[0]
        for ind in error_points:
            ind_search = ind
            while ind_search >= 0 and ind_search < 270 and scan[ind_search] - 0.001 < 1e-5:
                ind_search += 1
                if ind_search == 270:
                    ind_search = 0
            scan[ind] = scan[ind_search]
            
        time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        
        self.scan_record['time_stamp'].append(time_stamp)
        self.scan_record['scan'].append(scan)
        
    def odom_callback(self, msg):
        odom_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        linear_speed = msg.twist.twist.linear.x
        angular_speed = msg.twist.twist.angular.z
        
        if isinstance(self.pre_odom, np.ndarray):
            delta_time = odom_time - self.pre_odom[2]
            self.odom_record['delta_time'].append(delta_time)
            self.odom_record['time_stamp'].append(odom_time)
            self.odom_record['odom'].append([linear_speed, angular_speed])
            self.pre_odom = np.array([linear_speed, angular_speed, odom_time])
        else:
            self.pre_odom = np.array([linear_speed, angular_speed, odom_time])
        

def main(args=None):
    rclpy.init(args=args)
    trajectory_record_node = TrajectoryRecord()
    try:
        rclpy.spin(trajectory_record_node)
    except KeyboardInterrupt:
        pass
        # np.savez_compressed('traj_record', trajectory_record=trajectory_record_node.trajectory_record)
        # np.savez_compressed('scan_record', scan_record=trajectory_record_node.scan_record)
        # np.savez_compressed('odom_record', odom_record=trajectory_record_node.odom_record)

    trajectory_record_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
