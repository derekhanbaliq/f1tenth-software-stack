#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        
        # Topics & Subs, Pubs
        lidar_scan_topic = '/scan'
        pp_point_topic = '/pp_point'
        gf_point_topic = '/gf_point'
        vis_gf_marker_topic = '/vis_raw_gf_marker'
        vis_gap_fan_marker_topic = '/vis_gap_fan_marker_arr'

        # Subscribe to LIDAR
        self.sub_scan = self.create_subscription(LaserScan, lidar_scan_topic, self.lidar_callback, 10)
        self.sub_scan  # prevent unused variable warning

        # for motion planning
        # Subscribe to the pure pursuit point
        self.sub_pp = self.create_subscription(Point, pp_point_topic, self.pure_pursuit_callback, 1)
        self.sub_pp  # prevent unused variable warning
        # Publish modified goal point in car frame to pure pursuit node
        self.pub_to_pp = self.create_publisher(Point, gf_point_topic, 1)
        
        # visualization of gap following point
        self.pub_vis_gf_point = self.create_publisher(Marker, vis_gf_marker_topic, 1)
        self.pub_vis_gap_fan = self.create_publisher(MarkerArray, vis_gap_fan_marker_topic, 1)

        # params
        self.downsample_gap = 10
        self.max_sight = 10.0
        self.bubble_radius = 2
        self.extender_thres = 0.5
        self.proc_ranges = np.zeros(72)
        self.max_gap_safe_dist = 1.2
        self.best_point_x = 0.0
        self.best_point_y = 0.0 
        self.obstacle = 0

    def pure_pursuit_callback(self, pp_point_msg):
        # receive the lookahead point
        pp_point_x = pp_point_msg.x
        pp_point_y = pp_point_msg.y
        print("pp point", pp_point_x, pp_point_y)

        if self.obstacle == 1:
            # publish the new point
            self.pub_to_pp.publish(Point(x = float(self.best_point_x), y = float(self.best_point_y), z = 0.0))
            self.obstacle = 0
        else:
            self.pub_to_pp.publish(Point(x = float(pp_point_x), y = float(pp_point_y), z = 0.0))

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
    
    def bubble_danger_zone(self, data, proc_ranges):

        bubble_radius = 17
        closest_point_index = np.argmin(proc_ranges)
        closest_point = np.min(proc_ranges)

        # OBSTACLE CONDITION
        walls_offset = 5
        if closest_point < 1.0 and closest_point_index < len(proc_ranges)-walls_offset and closest_point_index > walls_offset:    # we do not want a bubble if the closest point is the wall on the sides
            self.obstacle = 1
            beginning_index = max(0,closest_point_index-bubble_radius)
            end_index = min(len(proc_ranges),closest_point_index+bubble_radius)
            proc_ranges[beginning_index:end_index] = 0.0

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

    def lidar_callback(self, data):
        """
        Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        # debugging setup
        np.set_printoptions(threshold=np.inf)

        # preprocessing & downsampling
        ranges = np.array(data.ranges[180:899])

        proc_ranges = self.preprocess_lidar(ranges)
        
        # bubble up
        self.proc_ranges = self.bubble_danger_zone(data, proc_ranges)

        # find max length gap 
        if self.obstacle == 1:
            start_max_gap, end_max_gap = self.find_max_gap(self.proc_ranges)
            # visualize max gap
            self.vis_fan(start_max_gap, end_max_gap)

            # find the best point in the gap 
            best_i = self.find_best_point(start_max_gap, end_max_gap, proc_ranges)
            degrees = best_i / 4.0
            radians = degrees * (np.pi/180)
            distance = self.proc_ranges[int(best_i)]
            self.best_point_x = np.cos(radians) * distance
            self.best_point_y = np.sin(radians) * distance

            # visualize best point
            self.vis_raw_gf_point(best_i)

    def vis_raw_gf_point(self, best_i):
        best_i_angle = np.deg2rad(180 / len(self.proc_ranges) * best_i)
        best_i_x = np.sin(best_i_angle)
        best_i_y = -np.cos(best_i_angle)

        # yellow
        gf_point_marker = Marker()
        gf_point_marker.header.frame_id = 'ego_racecar/base_link'  # if global then 'map'
        gf_point_marker.type = Marker.POINTS
        gf_point_marker.color.r = 0.75
        gf_point_marker.color.g = 0.75
        gf_point_marker.color.a = 1.0
        gf_point_marker.scale.x = 0.2
        gf_point_marker.scale.y = 0.2
        gf_point_marker.id = 6

        gf_point_marker.points = [Point(x = best_i_x, y = best_i_y, z = 0.2)]
        
        self.pub_vis_gf_point.publish(gf_point_marker)

    def vis_fan(self, start_max_gap, end_max_gap):
        fan_marker_arr = MarkerArray()
        fan_marker_arr.markers = []

        # start line
        fan_start_angle = np.deg2rad(180 / len(self.proc_ranges) * start_max_gap)
        fan_start_x = np.sin(fan_start_angle)
        fan_start_y = -np.cos(fan_start_angle)

        fan_start_point_marker = Marker()
        fan_start_point_marker.header.frame_id = 'ego_racecar/base_link'  # if global then 'map'
        fan_start_point_marker.type = Marker.LINE_STRIP
        fan_start_point_marker.color.b = 0.75
        fan_start_point_marker.color.g = 0.75
        fan_start_point_marker.color.a = 1.0
        fan_start_point_marker.scale.x = 0.04
        fan_start_point_marker.scale.y = 0.04
        fan_start_point_marker.id = 4

        fan_start_point_marker.points = [Point(x = fan_start_x, y = fan_start_y, z = 0.2), Point(x = 0., y = 0., z = 0.2)]
        
        fan_marker_arr.markers.append(fan_start_point_marker)

        # end line
        fan_end_angle = np.deg2rad(180 / len(self.proc_ranges) * end_max_gap)
        fan_end_x = np.sin(fan_end_angle)
        fan_end_y = -np.cos(fan_end_angle)

        fan_end_point_marker = Marker()
        fan_end_point_marker.header.frame_id = 'ego_racecar/base_link'  # if global then 'map'
        fan_end_point_marker.type = Marker.LINE_STRIP
        fan_end_point_marker.color.r = 0.
        fan_end_point_marker.color.g = 0.
        fan_end_point_marker.color.b = 0.
        fan_end_point_marker.color.g = 0.
        fan_end_point_marker.color.a = 1.0
        fan_end_point_marker.scale.x = 0.04
        fan_end_point_marker.scale.y = 0.04
        fan_end_point_marker.id = 5

        fan_end_point_marker.points = [Point(x = fan_end_x, y = fan_end_y, z = 0.2), Point(x = 0., y = 0., z = 0.2)]
        
        fan_marker_arr.markers.append(fan_end_point_marker)

        self.pub_vis_gap_fan.publish(fan_marker_arr)


def main(args=None):
    rclpy.init(args=args)
    print("Gap Follow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

