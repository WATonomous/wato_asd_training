from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description():
    """Launch transformer node."""
    transformer_pkg_prefix = get_package_share_directory('occupancy')

    node = Node(
        package='occupancy',
        executable='occupancy_node'
    )

    return LaunchDescription([
        node
    ])
