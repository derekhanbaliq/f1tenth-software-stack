from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    # joystick
    remoter_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('f1tenth_stack'), 'launch'), '/bringup_launch.py'
            ]
        )
    )

    # particle filter
    pf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('particle_filter'), 'launch'), '/localize_launch.py'
            ]
        )
    )

    # simulation
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
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
    
    ld.add_action(remoter_bringup_launch)
    ld.add_action(pf_launch)
    ld.add_action(rviz_node)
    # ld.add_action(pure_pursuit_node)
    # ld.add_action(lqr_node)
    ld.add_action(mpc_node)

    return ld
    
