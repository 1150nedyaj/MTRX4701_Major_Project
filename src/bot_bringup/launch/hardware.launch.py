from os import path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():
    ###  --- Group A ---  ###
    # Turtlebot 
    tb_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(tb_bringup_dir, 'launch', 'robot.launch.py')
        )
    )    
    camera_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(tb_bringup_dir, 'launch', 'camera.launch.py')
        )
    )

    # mmWave Radar 
    radar_dir = get_package_share_directory('mmwave_radar')
    radar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(radar_dir, 'launch', 'array.bringup.launch.py')
        )
    )

    group_a = TimerAction(
        period=0.0,
        actions=[robot_bringup, camera_bringup, radar_bringup]
    )

    ### --- Group B --- ###
    # Lidar People Detection 
    lidar_people_node = Node(
        package='lidar_radar',
        executable='people_detect',
        name="people_detect",
        output="screen"
    )

    # Sensor Fusion 
    fusion_node = Node(
        package='sensor_fusion',
        executable='sensor_fusion',
        name="sensor_fusion",
        output="screen"
    )

    # QR Destinations
    qr_dir = get_package_share_directory('qr_destinations')
    qr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(qr_dir, 'launch', 'destination_scanner.bringup.launch.py')
        )
    )

    group_b = TimerAction(
        period=10.0,
        actions=[lidar_people_node, qr_launch] # , fusion_node
    )

    launch_seq = [
        group_a,
        group_b
    ]

    return LaunchDescription(launch_seq)

