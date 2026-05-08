from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    is_real = LaunchConfiguration("is_real")

    declare_is_real = DeclareLaunchArgument(
        "is_real",
        default_value="false",
        description="Set true for real robot/live sensors, false for simulation or bag replay.",
    )

    simulation_group = GroupAction(
        condition=UnlessCondition(is_real),
        actions=[
            LogInfo(msg="Launching in SIMULATION / BAG mode"),

            # Placeholder for simulation/bag-mode nodes.
            # Later this can include Gazebo, RViz, bag replay, or fake people.
        ],
    )

    real_group = GroupAction(
        condition=IfCondition(is_real),
        actions=[
            LogInfo(msg="Launching in REAL ROBOT mode"),

            # Placeholder for real-robot nodes.
            # Later this can include real mmWave driver, real sensor setup, etc.
        ],
    )

    lidar_node = Node(
        package="lidar_radar",
        executable="lidar_detector_node",
        name="lidar_detector",
        output="screen",
    )

    mmwave_node = Node(
        package="mmwave_radar",
        executable="mmwave_node",
        name="mmwave_radar",
        output="screen",
    )

    movement_node = Node(
        package="robot_movement",
        executable="movement_node",
        name="robot_movement",
        output="screen",
    )

    return LaunchDescription([
        declare_is_real,
        simulation_group,
        real_group,
        lidar_node,
        mmwave_node,
        movement_node,
    ])
