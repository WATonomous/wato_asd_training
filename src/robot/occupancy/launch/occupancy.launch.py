from launch import LaunchDescription
from launch_ros.actions import Node

import os

def generate_launch_description():
    occupancy_node = Node(
        package='occupancy',
        name='occupancy_node',
        executable='occupancy_node',
        output='screen'
    )

    return LaunchDescription([
        occupancy_node
    ])
