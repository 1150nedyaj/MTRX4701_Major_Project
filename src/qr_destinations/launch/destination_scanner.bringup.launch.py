import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue




def generate_launch_description():

    return LaunchDescription([
        Node(
        package="qr_destinations",
        executable="destination_node.py",
        name="destination_advertiser",
        output="screen",
    )
    ])