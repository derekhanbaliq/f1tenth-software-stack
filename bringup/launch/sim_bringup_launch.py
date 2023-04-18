from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
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
    )

    pure_pursuit_node = Node(
        package="pure_pursuit",
        executable="motion_planning_pure_pursuit_node.py"
    )
    
    ld.add_action(gym_bridge_launch)
    ld.add_action(gap_follow_node)
    ld.add_action(pure_pursuit_node)

    return ld
