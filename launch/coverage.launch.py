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
    calculator = LaunchConfiguration('calculator').perform(context)
    param_file = os.path.join(get_package_share_directory('farmbot_trailblazer'), 'config', 'params.yaml')

    nodes_array = []
    is_calculator = False if calculator == '0' else True
    # print({'alternate_freq': alternate_freq} if alternate_freq != '' else {})

    gen_lines = Node(
        package='farmbot_trailblazer',
        executable="gen_lines",
        name='gen_lines',
        namespace=namespace,
        parameters=[
            yaml.safe_load(open(param_file))['gen_lines']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters'],
            {'path_angle': float(path_angle)} if path_angle != '' else {},
            {'alternate_freq': int(alternate_freq)} if alternate_freq != '' else {},
        ]
    )
    if is_calculator: nodes_array.append(gen_lines)

    getthe_field = Node(
        package='farmbot_trailblazer',
        executable="getthe_field",
        name='getthe_field',
        namespace=namespace,
        parameters=[
            yaml.safe_load(open(param_file))['getthe_field']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]
    )
    if is_calculator: nodes_array.append(getthe_field)

    to_nav = Node(
        package='farmbot_trailblazer',
        executable="to_nav",
        name='to_nav',
        namespace=namespace,
        parameters=[
            yaml.safe_load(open(param_file))['to_nav']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters']
        ]
    )
    nodes_array.append(to_nav)

    return nodes_array


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument('namespace', default_value='fbot')
    path_angle = DeclareLaunchArgument('path_angle', default_value='')
    alternate_freq = DeclareLaunchArgument('alternate_freq', default_value='')
    calculator = DeclareLaunchArgument('calculator', default_value='0')

    return LaunchDescription([
            namespace_arg,
            path_angle,
            alternate_freq,
            calculator,
            OpaqueFunction(function = launch_setup)
        ]
    )
