#!/usr/bin/env python3
"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid

# TODO: import as you need
from scipy.spatial import transform
from visualization_msgs.msg import Marker, MarkerArray
import matplotlib.pyplot as plt


class RRTNode(object):
    # class def for tree nodes
    def __init__(self, pos, parent, is_root):
        self.pos = pos # np array [x, y]
        self.parent = parent
        self.cost = None  # only used in RRT*
        self.is_root = is_root
    
    def __str__(self):
        return f"{self.pos}"
    
# def RRTTree(object):
#     def __init__(self, root):
#         self.root = root


class RRTGraph:
    def __init__(self, start_pos, graph_vis):
        ## implemented using adjacency lists
        self.vertices = []
        self.g = {} # [None] * len(self.vertices)
        # self.graph_vis = np.zeros((61, 100))
        self.graph_vis = graph_vis * 20  # better plotting

        self.start_node = RRTNode(start_pos, None, True)
        self.start_node.cost = 0.0
        self.addVertex(self.start_node)
        # self.end_node = RRTNode(end_pos, self.start_node, False)
        # self.addEdge(self.start_node, self.end_node)

    def addEdge(self, src_vertex, dest_vertex):
        self.addVertex(src_vertex)
        self.addVertex(dest_vertex)

        self.g[src_vertex].append(dest_vertex)
        self.g[dest_vertex].append(src_vertex)

    def removeEdge(self, src_vertex, dest_vertex):
        if dest_vertex in self.g[src_vertex]:
            self.g[src_vertex].remove(dest_vertex)
        if src_vertex in self.g[dest_vertex]:
            self.g[dest_vertex].remove(src_vertex)

    def addVertex(self, vertex):
        if vertex not in self.vertices:
            self.vertices.append(vertex)
            self.g[vertex] = []
    
    def visualize_path_in_matplotlib(self, start, path, goal):
        # visualize
        for i, node in enumerate(path):
            self.graph_vis[node.pos[0], node.pos[1]] = i * 2 + 4
        
        self.graph_vis[start[0], start[1]] = 20
        self.graph_vis[goal[0], goal[1]] = 20

        plt.matshow(self.graph_vis)
        plt.colorbar()
        plt.show()
    
    def __str__(self):
        s = ""
        s += "["
        for v in self.vertices:
            s += f"{v.pos}, "
        s += "]"
        return s
        

class OccGridMap(object):
    # class def for the occupancy grid map
    def __init__(self, res, width, depth):
        self.res = res
        self.w = width
        self.d = depth
        self.w_in_grid = int(round(self.w / self.res)) + 1  # x grid
        self.d_in_grid = int(round(self.d / self.res))  # y grid
        self.grid_map = np.zeros((self.w_in_grid, self.d_in_grid)).astype(np.int8)
        self.is_updated = False

    def update_occ_grid_map(self, obs_data_local):
        # reset map first
        self.grid_map = np.zeros_like(self.grid_map)

        # fill the occupancy
        # print(obs_data_local.shape)
        for i in range(obs_data_local.shape[1]):  # 3 x 108
            y = int(round(obs_data_local[0, i] / self.res))  # longitudial - that's because car's forward dir is x positive dir!
            x = -int(round(obs_data_local[1, i] / self.res))  # lateral
            # print("x = {} & y = {}".format(x, y))
            if -self.w_in_grid / 2 <= x <= self.w_in_grid / 2 and 0 <= y < self.d_in_grid:
                self.grid_map[int(x + self.w_in_grid // 2), int(y)] = 1
                # for j, k in ((-1, 0), (0, -1), (0, 1), (1, 0)):  # cross
                # for j, k in ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)):  # square / surronding 8
                for j in [-3, -2, -1, 0, 1, 2, 3]:
                    for k in [-3, -2, -1, 0, 1, 2, 3]:  # square / surrounding 49 ~ around half car
                        point_x = np.clip(x + j, -(self.w_in_grid // 2), self.w_in_grid // 2)
                        point_y = np.clip(y + k, 0, self.d_in_grid - 1)
                        # print("point_x = {}, point_y = {}".format(point_x, point_y))
                        self.grid_map[int(point_x + self.w_in_grid // 2), int(point_y)] = 1

        # visualize the occupancy grid map matrix in matplotlib
        # np.set_printoptions(threshold=np.inf)
        # print(self.grid_map)
        # plt.matshow(self.grid_map)
        # plt.colorbar()
        # plt.show()

        self.is_updated = True

    def get_local_obs_data_from_grid(self):
        occ_data_local = []
        
        for i in range(self.grid_map.shape[0]):  # 61
            for j in range(self.grid_map.shape[1]):  # 100
                if self.grid_map[i, j] == 1:
                    x = (i - self.w_in_grid // 2) * self.res
                    y = j * self.res
                    occ_data_local.append(np.array([y, -x, 0.0]))  # change the dir
        
        occ_data_local = (np.asarray(occ_data_local)).T
        # print(occ_data_local.shape)
        
        return occ_data_local
    
    def get_local_path_data_from_grid(self, path):
        path_data_local = []

        for i in range(path.shape[0]):
            x = (path[i, 0] - self.w_in_grid // 2) * self.res
            y = path[i, 1] * self.res
            path_data_local.append(np.array([y, -x, 0.0]))  # change the dir

        path_data_local = (np.asarray(path_data_local)).T

        return path_data_local
    
    def get_pp_point_into_occ_grid(self, point):
        x = np.clip(-int(round(point[1] / self.res)), -(self.w_in_grid // 2), self.w_in_grid // 2)
        y = np.clip(int(round(point[0] / self.res)), 0, self.d_in_grid - 1)

        return np.array([x + self.w_in_grid // 2, y])
    
    def get_local_rrt_point_from_grid(self, point):
        x = point[1] * self.res
        y = -(point[0] - self.w_in_grid // 2) * self.res

        return np.array([x, y])


class RRT(Node):
    # class def for RRT
    def __init__(self):
        super().__init__('rrt_node')

        np.set_printoptions(precision=2)
        # sim or real
        self.is_real = False
        self.RRT_star = True

        pose_topic = "/pf/viz/inferred_pose" if self.is_real else "/ego_racecar/odom"  # pose_topic = "ego_racecar/odom"
        scan_topic = "/scan"
        # laser_marker_topic = "/laser_marker"
        obs_marker_topic = "/obs_marker"
        # tree_marker_topic = "/tree_marker"
        edge_marker_topic = "/edge_marker_array"
        path_marker_topic = "/path_marker"
        pp_goal_topic = "/pp_goal"
        rrt_goal_topic = '/rrt_goal'

        # create subscribers and publishers
        self.pose_sub = self.create_subscription(PoseStamped if self.is_real else Odometry, pose_topic, self.pose_callback, 1)
        self.pose_sub
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 1)
        self.scan_sub
        # Publish goal point in car frame to RRT node
        self.pub_to_pp = self.create_publisher(Point, rrt_goal_topic, 1)

        self.pp_goal_point_sub = self.create_subscription(Point, pp_goal_topic, self.pp_callback, 1)
        self.pp_goal_point_sub

        # create other publishers that you might need
        # self.laser_marker_pub = self.create_publisher(Marker, laser_marker_topic, 1)  # for debugging
        # self.laser_marker_msg = Marker()
        self.obs_marker_pub = self.create_publisher(Marker, obs_marker_topic, 1)
        self.obs_marker_msg = Marker()
        # self.tree_marker_pub = self.create_publisher(Marker, tree_marker_topic, 1)
        # self.tree_marker_msg = Marker()
        self.edge_marker_pub = self.create_publisher(MarkerArray, edge_marker_topic, 1)
        self.path_marker_pub = self.create_publisher(Marker, path_marker_topic, 1)
        self.path_marker_msg = Marker()

        # class attributes, maybe create your occupancy grid here
        # self.downsample_gap = 1  # no downsampling
        self.occ_grid_map = OccGridMap(res=0.05, width=5.0, depth=5.0)  # local map 3.0m x 5.0m, res 0.05m/grid

        # init state - avoid unknown variables for scan callback
        self.curr_pos = np.array([0.0, 0.0, 0.0])
        self.rot_mat = np.identity(3)

        self.start = np.array([self.occ_grid_map.w_in_grid // 2, 0])  # (30. 0)
        self.goal = np.array([30, 50])  # init pos for goal

        # rrt params
        self.step_size = 10.0
        self.goal_thresh = 10.0
        self.point_num = 3

        # rrt* params
        self.c = 1.0
        self.radius = 25.0  #needs tuning

    # callback functions
    def pp_callback(self, pp_goal_msg):
        # print(f'received pp goal message:',pp_goal_msg.x)
        pp_point = np.array([pp_goal_msg.x, pp_goal_msg.y])  # in car frame
        # print("car frame = ", pp_point)

        # TODO: process of the point
        pp_point_grid = self.occ_grid_map.get_pp_point_into_occ_grid(pp_point)  # in grid map
        # print(pp_point_grid)

        self.goal = pp_point_grid

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        # polar coordinates of obs in car frame
        # proc_ranges = self.downsample_lidar(scan_msg)  # - ~ +, right to left
        proc_ranges = np.array(scan_msg.ranges)
        # print("processed ranges = {}".format(proc_ranges))
        proc_yaws = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(proc_ranges))
        # print("processed yaw angles = {}".format(proc_yaws))

        # cartesian coordinates in car frame
        obs_data_local = np.array([proc_ranges * np.cos(proc_yaws), proc_ranges * np.sin(proc_yaws)])
        obs_data_local = np.vstack((obs_data_local, np.zeros(len(proc_ranges))))  # 3 x 108, [x, y, 0.0]
        # print("local obs data = {}".format(obs_data_local))
        # print(obs_data_local.shape)
        
        # publish laser marker
        # self.visualize_laser_in_rviz(obs_data_local)

        # update the grid map
        self.occ_grid_map.update_occ_grid_map(obs_data_local)

        # publish obs marker
        self.visualize_obs_in_rviz()

    def pose_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        # update current pose and rotation matrix from the car frame to the world frame
        self.update_curr_pos(pose_msg)
        self.update_rot_mat(pose_msg)

        # if the grid map hasn't been init
        if self.occ_grid_map.is_updated == False:
            return 

        r_graph = RRTGraph(self.start, self.occ_grid_map.grid_map)
        
        # RRT Node is operating for 
        if not self.RRT_star:
            while True:
                x_rand = self.sample()
                x_nearest = self.nearest(r_graph, x_rand)
                x_nearest_node = r_graph.vertices[x_nearest]
                x_new_pos = self.steer(x_nearest_node.pos, x_rand)

                # create Node
                x_new = RRTNode(x_new_pos, x_nearest_node, False)

                if self.check_collision(x_nearest_node, x_new):
                    # add to Graph
                    r_graph.addEdge(x_nearest_node, x_new)

                # check if waypoint is hit
                if self.is_goal(x_new, self.goal):
                    break
            # visualize rrt tree vertices
            # self.visualize_tree_in_rviz(r_graph)
            # visualize rrt tree edges
            self.visualize_edge_in_rviz(r_graph)

            # get path & visualize
            path = self.find_path(r_graph, x_new)
            # r_graph.visualize_path_in_matplotlib(self.start, path, self.goal)
            path_mat = self.get_np_path(path)  # np version matrix
            lookahead_point = self.point_num if len(path_mat) > self.point_num else -1
            local_rrt_point = self.occ_grid_map.get_local_rrt_point_from_grid(path_mat[lookahead_point] if path_mat.shape[0] > 1 else path_mat[0])  # avoid single point
            # print(local_rrt_point)

            self.pub_to_pp.publish(Point(x = local_rrt_point[0], y = local_rrt_point[1], z = 0.0))

            self.visualize_path_in_rviz(path_mat)

        else:
            while True:
                x_rand = self.sample()
                x_nearest = self.nearest(r_graph, x_rand)
                x_nearest_node = r_graph.vertices[x_nearest]
                x_new_pos = self.steer(x_nearest_node.pos, x_rand)
                # create Node
                x_new = RRTNode(x_new_pos, x_nearest_node, False)

                if self.check_collision(x_nearest_node, x_new):
                    X_near = self.near(r_graph, x_new)

                    x_min = x_nearest_node
                    c_min = self.cost(r_graph, x_nearest_node) + self.c*self.line_cost(x_nearest_node, x_new)

                    # connect along a minimum-cost path
                    for x_near in X_near: 
                        if self.check_collision(x_new, x_near) and self.cost(r_graph, x_near)+self.c*self.line_cost(x_near, x_new) < c_min:
                            x_min = x_near
                            c_min = self.cost(r_graph, x_near)+self.c*self.line_cost(x_near, x_new)

                    # add to Graph
                    r_graph.addEdge(x_min, x_new)
                    
                    # rewire the tree
                    for x_near in X_near:
                        if self.check_collision(x_new, x_near) and self.cost(r_graph, x_new)+self.c*self.line_cost(x_near, x_new) < self.cost(r_graph, x_near):
                            x_parent = x_near.parent
                            r_graph.removeEdge(x_parent, x_near)
                            r_graph.addEdge(x_new, x_near)
                            x_near.parent = x_new

                # check if waypoint is hit
                if self.is_goal(x_new, self.goal):
                    break
            # visualize rrt tree vertices
            # self.visualize_tree_in_rviz(r_graph)
            # visualize rrt tree edges
            self.visualize_edge_in_rviz(r_graph)

            # get path & visualize
            path = self.find_path(r_graph, x_new)
            # r_graph.visualize_path_in_matplotlib(self.start, path, self.goal)
            path_mat = self.get_np_path(path)  # np version matrix
            lookahead_point = self.point_num if len(path_mat) > self.point_num else -1
            local_rrt_point = self.occ_grid_map.get_local_rrt_point_from_grid(path_mat[lookahead_point] if path_mat.shape[0] > 1 else path_mat[0])  # avoid single point
            # print(local_rrt_point)

            self.pub_to_pp.publish(Point(x = local_rrt_point[0], y = local_rrt_point[1], z = 0.0))

            self.visualize_path_in_rviz(path_mat)
        
    # additional functions used in callback for succinct expression    
    def downsample_lidar(self, scan_msg):
        """ 
        Preprocess the LiDAR scan array via downsampling by sthe pecific downsample gap.
        Modified from lab 4 - reactive gap following.
        """
        ranges = np.array(scan_msg.ranges)  # can be modified if needed
        proc_ranges = np.zeros(int(len(ranges) / self.downsample_gap))
        for i in range(len(proc_ranges)):
            proc_ranges[i] = sum(ranges[i * self.downsample_gap : (i + 1) * self.downsample_gap]) / self.downsample_gap
        
        # didn't clip the max sight this lab
        # max_sight = 5.0
        # proc_ranges = np.clip(proc_ranges, 0.0, max_sight)
        
        return proc_ranges

    def update_curr_pos(self, pose_msg):
        self.curr_x = pose_msg.pose.position.x if self.is_real else pose_msg.pose.pose.position.x
        self.curr_y = pose_msg.pose.position.y if self.is_real else pose_msg.pose.pose.position.y
        self.curr_pos = np.array([self.curr_x, self.curr_y, 0.0])
        # print("current pose = {}".format(self.curr_pos))

    def update_rot_mat(self, pose_msg):
        # get rotation matrix from the car frame to the world frame
        curr_orien = pose_msg.pose.orientation if self.is_real else pose_msg.pose.pose.orientation
        quat = [curr_orien.x, curr_orien.y, curr_orien.z, curr_orien.w]
        self.rot_mat = (transform.Rotation.from_quat(quat)).as_matrix()
        # print("rotation matrix = {}".format(self.rot_mat))

    # RRT functions
    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        while True:
            x = np.random.randint(0.0, self.occ_grid_map.grid_map.shape[0])
            y = np.random.randint(0.0, self.occ_grid_map.grid_map.shape[1])
            x, y = int(x), int(y)
            if self.occ_grid_map.grid_map[x, y] == 0: # if not occupied
                # print("Got a free sample")
                # print(x, y)
                break

        return np.array([x, y])

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_node = 0
        min_distance = float('inf')
        for i, node in enumerate(tree.vertices):
            node_dist = np.linalg.norm(node.pos - sampled_point)
            if node_dist < min_distance:
                min_distance = node_dist
                nearest_node = i
        # print("returning nearest node")

        return nearest_node

    def steer(self, nearest_node_pos, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node_pos nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        # new_node = None
        direction = (sampled_point - nearest_node_pos)
        direction = direction.astype(float)
        distance = np.linalg.norm(direction)
        # from IPython import embed; embed()
        direction *= min(distance, self.step_size) / (distance + 1e-6)
        
        new_node_pos = nearest_node_pos + direction
        new_node_pos = new_node_pos.astype(int)
        
        # print('steer')

        return new_node_pos

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        # get line segment between nodes
        # import pdb; pdb.set_trace()
        path = np.linspace(nearest_node.pos, new_node.pos)
        path = path.astype(int)
        
        # check each x, y in occupancy grid
        for x, y in path:
            if self.occ_grid_map.grid_map[x, y]: # if occupied
                return False

        return True # not collided

    def is_goal(self, latest_added_node, goal):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        distance = np.linalg.norm(latest_added_node.pos - goal)
        return False if distance > self.goal_thresh else True

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        curr = latest_added_node
        while (curr.pos != tree.start_node.pos).any():
            path.append(curr)
            curr = curr.parent
        path.append(curr)
        # from IPython import embed; embed()

        return path[::-1]

    def get_np_path(self, path):
        # change path from node list to np array
        path_mat = np.zeros((len(path) + 1, 2))
        for i, node in enumerate(path):
            # print(node.pos)
            path_mat[i, 0] = node.pos[0]
            path_mat[i, 1] = node.pos[1]
        path_mat[-1, 0] = self.goal[0]
        path_mat[-1, 1] = self.goal[1]
        # print(path_mat)

        return path_mat

    # extra methods for RRT*
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node):S the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        if node.is_root:
            cost = 0
        else:
            cost = self.cost(tree, node.parent) + self.c*(self.line_cost(node.parent, node))
        return cost

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        eucl_dist = np.linalg.norm(n1.pos - n2.pos)
        return eucl_dist

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []

        for temp_node in tree.vertices:
            if np.linalg.norm(temp_node.pos - node.pos) < self.radius:
                neighborhood.append(temp_node)
        
        return neighborhood

    # publish markers in rviz for visualization
    def visualize_laser_in_rviz(self, obs_data_local):
        # transform the obs data to the world frame
        obs_data_world = (self.curr_pos.reshape(3, 1) + self.rot_mat @ obs_data_local).T
        # print("world obs data = {}".format(obs_data_world))
        # print(obs_data_world.shape)

        # visualize the laser data in the world frame
        self.laser_marker_msg.header.frame_id = '/map'
        self.laser_marker_msg.type = Marker.POINTS
        self.laser_marker_msg.color.r = 0.75
        self.laser_marker_msg.color.a = 1.0
        self.laser_marker_msg.scale.x = 0.2
        self.laser_marker_msg.scale.y = 0.2
        self.laser_marker_msg.id = 0
        for i in range(len(obs_data_world)):
            point = Point(x = obs_data_world[i][0], y = obs_data_world[i][1], z = obs_data_world[i][2])
            self.laser_marker_msg.points.append(point)
        
        self.laser_marker_pub.publish(self.laser_marker_msg)

    def visualize_obs_in_rviz(self):
        # get the local obs data from the gridding process, then switch to world obs data
        obs_data_local = self.occ_grid_map.get_local_obs_data_from_grid()
        obs_data_world = (self.curr_pos.reshape(3, 1) + self.rot_mat @ obs_data_local).T

        # visualize the obs data in the world frame, after gridding process
        self.obs_marker_msg.points = []
        self.obs_marker_msg.header.frame_id = '/map'
        self.obs_marker_msg.type = Marker.POINTS
        self.obs_marker_msg.color.r = 0.75
        self.obs_marker_msg.color.a = 1.0
        self.obs_marker_msg.scale.x = 0.04
        self.obs_marker_msg.scale.y = 0.04
        self.obs_marker_msg.id = 1
        for i in range(len(obs_data_world)):
            point = Point(x = obs_data_world[i][0], y = obs_data_world[i][1], z = obs_data_world[i][2])
            self.obs_marker_msg.points.append(point)
        
        self.obs_marker_pub.publish(self.obs_marker_msg)

    def visualize_tree_in_rviz(self, r_graph):
        tree = []
        for i in range(len(r_graph.vertices)):
            # print(r_graph.vertices[i].pos)
            tree.append(r_graph.vertices[i].pos)
        tree_mat = np.asarray(tree)
        
        # get the local path data from the gridding process, then switch to world path data
        tree_data_local = self.occ_grid_map.get_local_path_data_from_grid(tree_mat)
        tree_data_world = (self.curr_pos.reshape(3, 1) + self.rot_mat @ tree_data_local).T

        # visualize the path data in the world frame, after gridding process
        self.tree_marker_msg.points = []
        self.tree_marker_msg.header.frame_id = '/map'
        self.tree_marker_msg.type = Marker.POINTS
        self.tree_marker_msg.color.g = 0.75
        self.tree_marker_msg.color.a = 1.0
        self.tree_marker_msg.scale.x = 0.05
        self.tree_marker_msg.scale.y = 0.05
        self.tree_marker_msg.id = 2
        for i in range(len(tree_data_world)):
            point = Point(x = tree_data_world[i][0], y = tree_data_world[i][1], z = tree_data_world[i][2])
            self.tree_marker_msg.points.append(point)
        
        self.tree_marker_pub.publish(self.tree_marker_msg)

    def visualize_edge_in_rviz(self, r_graph):
        edge_marker_arr = MarkerArray()
        edge_marker_arr.markers = []
        # visualize rrt edges
        for i in range(len(r_graph.vertices)):
            edge = []
            tail = r_graph.vertices[i].pos
            if r_graph.vertices[i].parent == None:
                continue
            head = r_graph.vertices[i].parent.pos
            # print(tail, head)
            edge.append(tail)
            edge.append(head)
            edge_mat = np.asarray(edge)
            # print(edge_mat)

            edge_data_local = self.occ_grid_map.get_local_path_data_from_grid(edge_mat)
            edge_data_world = (self.curr_pos.reshape(3, 1) + self.rot_mat @ edge_data_local).T
            
            edge_marker = Marker()
            edge_marker.header.frame_id = '/map'
            edge_marker.type = Marker.LINE_STRIP
            edge_marker.color.b = 1.0
            edge_marker.color.g = 1.0
            edge_marker.color.a = 1.0
            edge_marker.scale.x = 0.02
            edge_marker.scale.y = 0.02
            edge_marker.id = i + 5

            for i in range(len(edge_data_world)):
                point = Point(x = edge_data_world[i][0], y = edge_data_world[i][1], z = edge_data_world[i][2])
                edge_marker.points.append(point)

            edge_marker_arr.markers.append(edge_marker)

        self.edge_marker_pub.publish(edge_marker_arr)

    def visualize_path_in_rviz(self, path_mat):
        # get the local path data from the gridding process, then switch to world path data
        path_data_local = self.occ_grid_map.get_local_path_data_from_grid(path_mat)
        path_data_world = (self.curr_pos.reshape(3, 1) + self.rot_mat @ path_data_local).T
        # visualize the path data in the world frame, after gridding process
        self.path_marker_msg.points = []
        self.path_marker_msg.header.frame_id = '/map'
        self.path_marker_msg.type = Marker.LINE_STRIP  # as line segments
        self.path_marker_msg.color.b = 0.75
        self.path_marker_msg.color.a = 1.0
        self.path_marker_msg.scale.x = 0.1
        self.path_marker_msg.scale.y = 0.1
        self.path_marker_msg.id = 3
        for i in range(len(path_data_world)):
            point = Point(x = path_data_world[i][0], y = path_data_world[i][1], z = path_data_world[i][2])
            self.path_marker_msg.points.append(point)
        
        self.path_marker_pub.publish(self.path_marker_msg)

def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRT()
    if rrt_node.RRT_star == True:
        print("RRT star Running")
    else:
        print("simple RRT Running")
    # print("RRT Initialized")
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
