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
    path_angle = LaunchConfiguration('path_angle').perform(context)
    alternate_freq = LaunchConfiguration('alternate_freq').perform(context)
    param_file = os.path.join(get_package_share_directory('farmbot_planner'), 'config', 'params.yaml')

    nodes_array = []

    print(alternate_freq)

    ab_planner = Node(
        package='farmbot_planner',
        executable="ab_planner",
        name='ab_planner',
        namespace=namespace,
        parameters=[
            yaml.safe_load(open(param_file))['ab_planner']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters'],
            {'path_angle': path_angle} if path_angle != '' else {},
            {'alternate_freq': alternate_freq} if alternate_freq != '' else {}
        ]
    )
    # nodes_array.append(ab_planner)

    goto_field = Node(
        package='farmbot_planner',
        executable="goto_field",
        name='goto_field',
        namespace=namespace,
        parameters=[
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
            yaml.safe_load(open(param_file))['getthe_field']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]
    )
    nodes_array.append(getthe_field)

    to_nav = Node(
        package='farmbot_planner',
        executable="to_nav",
        name='to_nav',
        namespace=namespace,
        parameters=[
            yaml.safe_load(open(param_file))['to_nav']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]
    )
    # nodes_array.append(to_nav)

    return nodes_array


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument('namespace', default_value='fbot')
    path_angle = DeclareLaunchArgument('path_angle', default_value='')
    alternate_freq = DeclareLaunchArgument('alternate_freq', default_value='')

    return LaunchDescription([
            namespace_arg,
            path_angle,
            alternate_freq,
            OpaqueFunction(function = launch_setup)
        ]
    )
