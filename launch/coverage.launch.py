import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import yaml
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    param_file = os.path.join(
        get_package_share_directory("farmbot_trailblazer"), "config", "params.yaml"
    )

    nodes_array = []

    bidder = Node(
        package="farmbot_trailblazer",
        executable="bidder",
        name="bidder",
        namespace=namespace,
        parameters=[
            yaml.safe_load(open(param_file))["global"]["ros__parameters"],
        ],
        output="screen",
    )
    nodes_array.append(bidder)

    generate = Node(
        package="farmbot_trailblazer",
        executable="generate",
        name="generate",
        namespace=namespace,
        parameters=[
            yaml.safe_load(open(param_file))["global"]["ros__parameters"],
        ],
        output="screen",
    )
    nodes_array.append(generate)

    divider = Node(
        package="farmbot_trailblazer",
        executable="divider",
        name="divider",
        namespace=namespace,
        parameters=[
            yaml.safe_load(open(param_file))["global"]["ros__parameters"],
        ],
        output="screen",
    )
    nodes_array.append(divider)

    return nodes_array


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument("namespace", default_value="fbot")

    return LaunchDescription(
        [
            namespace_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
