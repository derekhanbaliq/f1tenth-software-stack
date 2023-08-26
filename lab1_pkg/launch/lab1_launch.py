from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
#    config = os.path.join(
#        get_package_share_directory('lab1_pkg'),
#        'config',
#        'params.yaml'
#    )
    
    ld = LaunchDescription()

    talker_node = Node(
        package="lab1_pkg",
        executable="talker.py",
        parameters=[
            {"v": 6.0},
            {"d": 0.3}
        ]
    )

    relay_node = Node(
        package="lab1_pkg",
        executable="relay.py",
    )

    ld.add_action(relay_node)
    ld.add_action(talker_node)

    return ld
