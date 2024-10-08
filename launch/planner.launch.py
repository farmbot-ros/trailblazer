import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import yaml
import argparse
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    param_file = os.path.join(get_package_share_directory('farmbot_planner'), 'config', 'params.yaml')

    nodes_array = []

    robs4crops = bool(yaml.safe_load(open(param_file))['global']['ros__parameters']['r4c'])
    executable = 'path_server' if not robs4crops else 'path_server_r4c'
    path_server = Node(
        package='farmbot_navigation',
        executable=executable,
        name='path_server',
        namespace=namespace,
        parameters=[
            {"frame_prefix": namespace+"/"},
            {"namespace": namespace},
            yaml.safe_load(open(param_file))['path_server']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]
    )
    nodes_array.append(path_server)
    
    return nodes_array


def generate_launch_description(): 
    namespace_arg = DeclareLaunchArgument('namespace', default_value='fb')
    antena_arg = DeclareLaunchArgument('double_antenna', default_value='True')
    
    return LaunchDescription([
        namespace_arg,
        antena_arg, 
        OpaqueFunction(function = launch_setup)
        ]
    )