import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    is_real = LaunchConfiguration("is_real")

    declare_is_real = DeclareLaunchArgument(
        "is_real",
        default_value="false",
        description="false = launch Gazebo simulation, true = expect real robot or rosbag topics",
    )

    turtlebot3_gazebo_launch = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "launch",
        "turtlebot3_world.launch.py",
    )

    simulation_group = GroupAction(
        condition=UnlessCondition(is_real),
        actions=[
            LogInfo(msg="Launching TurtleBot3 Gazebo simulation"),
            SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="burger"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(turtlebot3_gazebo_launch)
            ),
        ],
    )

    real_or_bag_group = GroupAction(
        condition=IfCondition(is_real),
        actions=[
            LogInfo(
                msg=(
                    "REAL/BAG mode selected. "
                )
            ),
        ],
    )

    people_detect_node = Node(
        package="lidar_radar",
        executable="people_detect",
        name="people_detect",
        output="screen",
    )

    sensor_fusion_node = Node(
        package="sensor_fusion",
        executable="sensor_fusion",
        name="sensor_fusion",
        output="screen",
        parameters=[{
            # List every radar detection topic used in your array config.
            # Add more entries if you have multiple radar modules.
            "radar_topics": ["/mmWave_array/radar_0/detections"],
            # A lidar person is confirmed when a radar point is within this radius (metres).
            "fusion_distance_threshold": 1.0,
            # Radar readings older than this (seconds) are discarded from the buffer.
            # Increased from 0.5 s to tolerate short radar dropout gaps.
            "radar_timeout": 1.5,
            # A confirmed track is held in the output for this many seconds
            # after its last radar+lidar confirmation, smoothing out flickering.
            "hold_time": 1.0,
            # Common TF frame used for spatial comparison.
            "target_frame": "base_link",
        }],
    )

    return LaunchDescription([
        declare_is_real,
        simulation_group,
        real_or_bag_group,
        people_detect_node,
        sensor_fusion_node,
    ])