import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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

    return LaunchDescription([
        declare_is_real,
        simulation_group,
        real_or_bag_group,
    ])