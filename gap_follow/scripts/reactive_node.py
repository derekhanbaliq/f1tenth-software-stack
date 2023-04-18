#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Subscribe to LIDAR
        self.sub_scan = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.sub_scan  # prevent unused variable warning
        
        # Publish to drive
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.drive_msg = AckermannDriveStamped()
        
        # params
        self.downsample_gap = 10
        self.max_sight = 4.0
        self.bubble_radius = 2
        self.extender_thres = 0.5
        self.max_gap_safe_dist = 1.2

    def preprocess_lidar(self, ranges):
        """ 
        Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m) -- Derek: ????
        """
        proc_ranges = np.zeros(int(720 / self.downsample_gap))
        for i in range(len(proc_ranges)):
            proc_ranges[i] = sum(ranges[i * self.downsample_gap : (i + 1) * self.downsample_gap]) / self.downsample_gap
        
        proc_ranges = np.clip(proc_ranges, 0.0, self.max_sight)
        
        return proc_ranges

    def disparity_extender(self, proc_ranges):
        i = 0
        while i < len(proc_ranges) - 1:
            if proc_ranges[i + 1] - proc_ranges[i] >= self.extender_thres:
                proc_ranges[i : min(i + self.bubble_radius + 1, len(proc_ranges))] = proc_ranges[i]
                i += self.bubble_radius + 1
            elif proc_ranges[i] - proc_ranges[i + 1] >= self.extender_thres:
                proc_ranges[max(0, i - self.bubble_radius + 1) : i + 1] = proc_ranges[i + 1]  # use shorter to extend
                i += self.bubble_radius + 1
            else:
                i += 1

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ 
        Return the start index & end index of the max gap in free_space_ranges
        """
        longest_streak = 0
        streak = 0
        end_index = 0
        start_index = 0

        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] > self.max_gap_safe_dist: # != 0.0 leads to no obs
                streak += 1
                if (streak > longest_streak):
                    longest_streak = streak
                    end_index = i + 1  # the end notion begin with 1
                    start_index = end_index - longest_streak
            else:
                streak = 0

        return start_index, end_index
    
    def find_best_gap(self, start_i, end_i):

        return None

    def find_best_point(self, start_i, end_i, ranges):
        """
        Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        # deepest_index = np.argmax(ranges[start_i:end_i])
        # deepest_index += start_i
        best_index = (start_i + end_i) / 2

        return best_index

    def find_deepest_point(self, start_i, end_i, ranges):
        """
        Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        deepest_index = np.argmax(ranges[start_i:end_i])
        deepest_index += start_i

        return deepest_index

    def lidar_callback(self, data):
        """ 
        Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        # debugging setup
        np.set_printoptions(threshold=np.inf)

        # preprocessing & downsampling
        ranges = np.array(data.ranges[180:899])
        proc_ranges = self.preprocess_lidar(ranges)
        print("downsampling = ", proc_ranges)
        
        # bubble up
        # proc_ranges = self.bubble_danger_zone(data, proc_ranges)
        # proc_ranges = self.disparity_extender(proc_ranges)
        # print("extender = ", proc_ranges)

        # find max length gap 
        start_max_gap, end_max_gap = self.find_max_gap(proc_ranges)
        print('start_max_gap = ', start_max_gap)
        print('end_max_gap = ', end_max_gap)

        # find the best point in the gap 
        best_i = self.find_best_point(start_max_gap, end_max_gap, proc_ranges)
        # best_i = self.find_deepest_point(0, len(proc_ranges) - 1, proc_ranges)
        print(f'best point:', best_i)

        # map steering_angle
        steering_angle = np.deg2rad(best_i * self.downsample_gap / 4.0 - 90.0)

        # set the velocity
        # velocity = 0.5
        if sum(data.ranges[530:549]) / 20 < 2.0:
            velocity = 2.0
        # elif sum(data.ranges[530:549]) / 20 < 3.0:
        #     velocity = 3.5
        else:
            velocity = 5.0
        # if abs(self.drive_msg.drive.steering_angle) < np.pi / 18:
        #     velocity = 5.0
        # else:
        #     velocity = 2.0
        print(f'velocity:', velocity)

        # Publish Drive message
        self.drive_msg.drive.steering_angle = steering_angle
        self.drive_msg.drive.speed = velocity
        self.pub_drive.publish(self.drive_msg)

        # print(f'steering angle:', steering_angle)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

