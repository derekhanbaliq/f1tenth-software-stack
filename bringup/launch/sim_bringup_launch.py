from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    sim = False
    derek_map_path = os.path.realpath(os.path.join('src', 'map_data'))
    sofia_map_path = os.path.realpath(os.path.join('src', 'ESE-615-Final-Project', 'map_data'))

    ld = LaunchDescription()
    
    # simulation
    gym_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch'),
                '/gym_bridge_launch.py'
            ]
        )
    )

    gap_follow_node = Node(
        package="gap_follow",
        executable="motion_planning_reactive_node.py",
        parameters=[
            {"sim or real": sim},                           # do not change
            {"downsample gap": 10},                         # downsampling lidar values
            {"max sight": 10},                              # default to specific lidar
            {"disparity extender length": 2},               # not used
            {"disparity extender threshold": 0.5},          # not used
            {"safe distance of the max gap": 1.5},          # minimum distance of consecutive free rays to create max gap
            {"pure pursuit confidence ratio": 0.4},         # weight of pure pursuit versus gap follow
            {"lateral deviation threshold distance": 0.6},  # lateral deviation constraint (bubble)
        ],
        # output="screen"                                   # comment in for visible prints from gap follow
    )

    pure_pursuit_node = Node(
        package="pure_pursuit",
        executable="motion_planning_pure_pursuit_node.py",
        parameters=[
            {"sim or real": sim},                           # do not change
            {"is ascending": True},                         # direction of waypoints (True for ccw)
            {"map name": "skir_2_draw"},                    # map used
            {"map path": derek_map_path},                   # path of map in directory
            {"reference speed gain": 0.7},                  # weight of reference speed
            {"lookahead distance": 1.0},                    # lookahead of pure pursuit
            {"steering gain": 0.5},                         # steering gain of pure pursuit
            {"test speed": 1.0},                            # use for testing instead of reference speer
        ],
        output="screen"                                     # comment in for visible prints from pure pursuit           
    )
    
    # Note: the packages included in the launch command will not have convenient printing for debugging
    ld.add_action(gym_bridge_launch)                        # comment in to incorporate sim launch to the launch command
    ld.add_action(gap_follow_node)                          # comment in to incorporate gap follow to the launch command
    ld.add_action(pure_pursuit_node)                        # comment in to incorporate pure pursiot to the launch command

    return ld
