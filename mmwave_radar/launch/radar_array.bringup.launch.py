import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

default_parameter_file = 'v0_frame_solo_radar.yaml'

def generate_launch_description():
    node_list = []

    config_path = os.path.join(
        get_package_share_directory('mmwave_radar'),
        'config',
        default_parameter_file
    )

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    settings = config['launch_settings']
    modules = settings['radar_modules']

    for module in modules:
        module_node = Node(
            package="mmwave_radar",
            executable="radar_module_node.py",
            namespace="mmWave_array",
            name=f"radar_{module['identifier']}",
            output="screen",
            parameters=[
                {"identifier":module['identifier']},
                {"interface":module['interface']}
            ]
        )
        node_list.append(module_node)


    return LaunchDescription(
        node_list
    )