from os import path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    slam_tb_dir = get_package_share_directory('slam_toolbox')
    slam_tb_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(slam_tb_dir, 'launch', 'online_async_launch.py')
        )
    ) 

    bringup_dir = get_package_share_directory('bot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    params_file = path.join(bringup_dir, 'config', 'nav2_twistStamped.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items()
    ) 

    launch_seq = [
        slam_tb_launch,
        TimerAction(period=5.0,actions=[nav_launch])
    ]

    return LaunchDescription(launch_seq)
    