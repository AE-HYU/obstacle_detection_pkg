import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directory
    pkg_share = FindPackageShare('obstacle_detection_pkg')
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='obstacle_detection_params.yaml',
        description='Configuration file name'
    )
    
    # Configuration file path
    config_file_path = PathJoinSubstitution([
        pkg_share, 'config', LaunchConfiguration('config_file')
    ])
    
    # Map server node - use simple yaml_filename parameter
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': PathJoinSubstitution([
                pkg_share, 'maps', 'Spielberg_map.yaml'
            ]),
            'use_sim_time': False
        }],
        output='screen'
    )
    
    # Lifecycle manager for map server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{
            'node_names': ['map_server'],
            'use_sim_time': False,
            'autostart': True
        }],
        output='screen'
    )
    
    # Obstacle detection node
    obstacle_detector_node = Node(
        package='obstacle_detection_pkg',
        executable='obstacle_detector_node',
        name='obstacle_detector',
        parameters=[config_file_path],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        map_server_node,
        lifecycle_manager_node,
        obstacle_detector_node
    ])