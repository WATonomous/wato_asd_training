from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description():
    ld = LaunchDescription() # Begin building a launch description

    #################### Costmap Node #####################
    costmap_pkg_prefix = get_package_share_directory('costmap')
    costmap_param_file = os.path.join(
        costmap_pkg_prefix, 'config', 'params.yaml')
    
    costmap_param = DeclareLaunchArgument(
        'costmap_param_file',
        default_value=costmap_param_file,
        description='Path to config file for producer node'
    )
    costmap_node = Node(
        package='costmap',
        name='costmap_node',
        executable='costmap_node',
        parameters=[LaunchConfiguration('costmap_param_file')],
    )
    ld.add_action(costmap_param)
    ld.add_action(costmap_node)

    #################### Map Memory Node #####################
    map_memory_pkg_prefix = get_package_share_directory('map_memory')
    map_memory_param_file = os.path.join(
        map_memory_pkg_prefix, 'config', 'params.yaml')
    
    map_memory_param = DeclareLaunchArgument(
        'map_memory_param_file',
        default_value=map_memory_param_file,
        description='Path to config file for producer node'
    )
    map_memory_node = Node(
        package='map_memory',
        name='map_memory_node',
        executable='map_memory_node',
        parameters=[LaunchConfiguration('map_memory_param_file')],
    )
    ld.add_action(map_memory_param)
    ld.add_action(map_memory_node)
    
    ##################### Planner Node #####################
    planner_pkg_prefix = get_package_share_directory('planner')
    planner_param_file = os.path.join(
        planner_pkg_prefix, 'config', 'params.yaml')
    
    planner_param = DeclareLaunchArgument(
        'planner_param_file',
        default_value=planner_param_file,
        description='Path to config file for producer node'
    )
    planner_node = Node(
        package='planner',
        name='planner_node',
        executable='planner_node',
        parameters=[LaunchConfiguration('planner_param_file')],
    )
    ld.add_action(planner_param)
    ld.add_action(planner_node)
    
    ##################### Control Node #####################
    control_pkg_prefix = get_package_share_directory('control')
    control_param_file = os.path.join(
        control_pkg_prefix, 'config', 'params.yaml')
    
    control_param = DeclareLaunchArgument(
        'control_param_file',
        default_value=control_param_file,
        description='Path to config file for producer node'
    )
    control_node = Node(
        package='control',
        name='control_node',
        executable='control_node',
        parameters=[LaunchConfiguration('control_param_file')],
    )
    ld.add_action(control_param)
    ld.add_action(control_node)

    #################### Odometry Spoof Node #####################
    odometry_spoof_node = Node(
        package='odometry_spoof',
        name='odometry_spoof',
        executable='odometry_spoof',
    )
    ld.add_action(odometry_spoof_node)

    return ld

    #################### nav2 gps module #####################
    localization_pkg_prefix = get_package_share_directory('nav2_gps_waypoint_follower_demo')  # Use the package name from your Dockerfile
    gazebo_gps_launch_file = os.path.join(
        localization_pkg_prefix, 'launch', 'gazebo_gps_world.launch.py'
    )
    
    gazebo_gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_gps_launch_file),
    )
    ld.add_action(gazebo_gps_launch)
