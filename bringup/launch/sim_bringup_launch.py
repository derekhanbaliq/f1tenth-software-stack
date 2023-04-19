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
            {"sim or real": sim},
            {"downsample gap": 10},
            {"max sight": 10},
            {"disparity extender length": 2},
            {"disparity extender threshold": 0.5},
            {"safe distance of the max gap": 1.5},
            {"pure pursuit confidence ratio": 0.4},  # !!!!
            {"lateral deviation threshold distance": 0.6},  # lateral deviation constraint
        ],
        # output="screen"
    )

    pure_pursuit_node = Node(
        package="pure_pursuit",
        executable="motion_planning_pure_pursuit_node.py",
        parameters=[
            {"sim or real": sim},
            {"is ascending": True},
            {"map name": "skir_2_draw"},
            {"map path": derek_map_path},
            {"reference speed gain": 0.7},  # power!
            {"lookahead distance": 1.0},
            {"steering gain": 0.5},
            {"test speed": 1.0},
        ],
        output="screen"
    )
    
    ld.add_action(gym_bridge_launch)
    ld.add_action(gap_follow_node)
    ld.add_action(pure_pursuit_node)

    return ld
