from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    sim = False
    derek_csv_path = os.path.realpath(os.path.join('src', 'csv_data'))
    # sofia_csv_path = os.path.realpath(os.path.join('src', 'ESE-615-Final-Project', 'csv_data'))

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
        executable="final_race_reactive_node.py",
        parameters=[
            {"sim or real": sim},                           # do not change
            {"downsample gap": 10},                         # downsampling lidar values
            {"max sight": 10},                              # default to specific lidar
            {"disparity extender length": 2},               # not used
            {"disparity extender threshold": 0.5},          # not used
            {"safe distance of the max gap": 1.5},          # minimum distance of consecutive free rays to create max gap
            {"pure pursuit confidence ratio": 0.4},         # weight of pure pursuit versus gap follow
            {"lateral deviation threshold distance": 0.6},  # lateral deviation constraint (bubble)
            {"lookahead distance": 1.0},                    # lookahead of pure pursuit, keep gap following point has the same lookahead distance as the pure pursuit
            {"obstacle distance": 1.5},                     # how close the obstacle has to be to introduce gap follow to the waypoint calculation
        ],
        # output="screen"                                   # comment in for visible prints from gap follow
    )

    pure_pursuit_node = Node(
        package="pure_pursuit",
        executable="final_race_pure_pursuit_node.py",
        parameters=[
            {"sim or real": sim},                           # do not change
            {"is ascending": True},                         # direction of waypoints (True for ccw)
            {"csv name": "0420_best"},              # csv for map used, original - skir_2_draw
            {"csv path": derek_csv_path},                   # path of csv for map in directory
            {"reference speed gain": 0.5},                  # weight of reference speed, original - 0.7
            {"lookahead distance": 1.0},                    # lookahead of pure pursuit, original - 1.0
            {"steering gain": 0.45},                         # steering gain of pure pursuit, original - 0.5
            {"test speed": 1.0},                            # use for testing instead of reference speer
        ],
        output="screen"                                     # comment in for visible prints from pure pursuit           
    )
    
    # Note: the packages included in the launch command will not have convenient printing for debugging
    ld.add_action(gym_bridge_launch)                        # comment in to incorporate sim launch to the launch command
    ld.add_action(gap_follow_node)                          # comment in to incorporate gap follow to the launch command
    ld.add_action(pure_pursuit_node)                        # comment in to incorporate pure pursiot to the launch command

    return ld
