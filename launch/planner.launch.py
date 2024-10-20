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

    ab_planner = Node(
        package='farmbot_planner',
        executable="ab_planner",
        name='ab_planner',
        namespace=namespace,
        parameters=[
            {"frame_prefix": namespace+"/"},
            {"namespace": namespace},
            yaml.safe_load(open(param_file))['ab_planner']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]
    )
    # nodes_array.append(ab_planner)

    goto_field = Node(
        package='farmbot_planner',
        executable="goto_field",
        name='goto_field',
        namespace=namespace,
        parameters=[
            {"frame_prefix": namespace+"/"},
            {"namespace": namespace},
            yaml.safe_load(open(param_file))['goto_field']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]
    )
    nodes_array.append(goto_field)

    getthe_field = Node(
        package='farmbot_planner',
        executable="getthe_field",
        name='getthe_field',
        namespace=namespace,
        parameters=[
            {"frame_prefix": namespace+"/"},
            {"namespace": namespace},
            yaml.safe_load(open(param_file))['getthe_field']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]
    )
    nodes_array.append(getthe_field)
    
    return nodes_array


def generate_launch_description(): 
    namespace_arg = DeclareLaunchArgument('namespace', default_value='fb')

    return LaunchDescription([
            namespace_arg,
            OpaqueFunction(function = launch_setup)
        ]
    )