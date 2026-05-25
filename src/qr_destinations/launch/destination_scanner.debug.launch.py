from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
        package="qr_destinations",
        executable="debug_node.py",
        name="destination_debug",
        output="screen",
    )
    ])