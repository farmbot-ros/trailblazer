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
    param_file = os.path.join(get_package_share_directory('farmbot_planner'), 'config', 'params.yaml')

    nodes_array = []

    # print(alternate_freq)

    print({'alternate_freq': alternate_freq} if alternate_freq != '' else {})

    is_calculator = False if calculator == '0' else True

    gen_lines = Node(
        package='farmbot_planner',
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

    gen_path = Node(
        package='farmbot_planner',
        executable="gen_path",
        name='gen_path',
        namespace=namespace,
        parameters=[
            yaml.safe_load(open(param_file))['gen_path']['ros__parameters'],
            yaml.safe_load(open(param_file))['global']['ros__parameters'],
        ]
    )
    nodes_array.append(gen_path)

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
    if is_calculator: nodes_array.append(getthe_field)

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
    calculator = DeclareLaunchArgument('calculator', default_value='0')

    return LaunchDescription([
            namespace_arg,
            path_angle,
            alternate_freq,
            calculator,
            OpaqueFunction(function = launch_setup)
        ]
    )
