from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    real = True
    derek_map_path = os.path.realpath(os.path.join('src', 'map_data'))  # default path as weel
    sofia_map_path = os.path.realpath(os.path.join('src', 'ESE-615-Final-Project', 'map_data'))

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
    
    gap_follow_node = Node(
        package="gap_follow",
        executable="motion_planning_reactive_node.py",
        parameters=[
            {"sim or real": real},                          # do not change
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
            {"sim or real": real},                          # do not change
            {"is ascending": True},                         # direction of waypoints (True for ccw)
            {"map name": "skir_2_draw"},                    # map used
            {"map path": derek_map_path},                   # path of map in directory
            {"reference speed gain": 0.6},                  # weight of reference speed
            {"lookahead distance": 2.2},                    # lookahead of pure pursuit
            {"steering gain": 0.45},                        # steering gain of pure pursuit
            {"test speed": 1.0},                            # use for testing instead of reference speer
        ],
        # output="screen"                                   # comment in for visible prints from pure pursuit 
    )                                    
    
    # Note: the packages included in the launch command will not have convenient printing for debugging
    ld.add_action(remoter_bringup_launch)                    # comment in to incorporate joystick control to the launch command
    ld.add_action(pf_launch)                                # comment in to incorporate particle filter to the launch command
    ld.add_action(rviz_node)                                # comment in to incorporate sim launch to the launch command
    ld.add_action(gap_follow_node)                          # comment in to incorporate gap follow to the launch command
    ld.add_action(pure_pursuit_node)                        # comment in to incorporate pure pursiot to the launch command

    return ld
