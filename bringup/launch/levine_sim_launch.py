from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
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

    pure_pursuit_node = Node(
        package="pure_pursuit",
        executable="pure_pursuit_node.py",  # please modify the params first!
        output="screen",
    )

    lqr_node = Node(
        package="lqr",
        executable="lqr_node.py",  # please modify the params first!
        output="screen",
    )

    mpc_node = Node(
        package="mpc",
        executable="mpc_node.py",  # please modify the params first!
        output="screen",
    )
    
    ld.add_action(gym_bridge_launch)
    # ld.add_action(pure_pursuit_node)
    ld.add_action(lqr_node)
    # ld.add_action(mpc_node)

    return ld
