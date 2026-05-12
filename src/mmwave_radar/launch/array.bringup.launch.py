import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

default_parameter_file = 'v0_frame_solo_radar.yaml'

def setup_routine(context):
    node_list = []

    ## Get config data
    parameter_file = LaunchConfiguration("config").perform(context)

    print(f"\t\t\t### Launching Array with {parameter_file} config ###")

    config_path = os.path.join(
        get_package_share_directory('mmwave_radar'),
        'config',
        parameter_file
    )

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)


    ## Static transform broadcaster for nodes
    static_transform_node = Node(
        package="mmwave_radar",
        executable="array_transforms_node.py",
        name="array_tf_publisher",
        output="screen",
        arguments=[str(config_path)]
    )
    node_list.append(static_transform_node)


    ## Nodes for each radar
    modules = config['launch_settings']['radar_modules']
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

    return node_list


def generate_launch_description():
    array_config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_parameter_file,
        description='Radar semantics, interfaces and physical orientations'
    )


    return LaunchDescription([
        array_config_arg,
        OpaqueFunction(function=setup_routine)
    ])