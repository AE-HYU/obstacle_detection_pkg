#!/usr/bin/env python3
"""
Obstacle Detection Package Launch File

This launch file starts the complete obstacle detection system including:
1. Map server for static environment map
2. Lifecycle manager to control map server state  
3. Obstacle detector node for real-time dynamic obstacle detection

The system uses DBSCAN clustering to detect dynamic obstacles from LiDAR data
while filtering out static map obstacles.

Author: AI Lab Team
License: MIT
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate launch description for obstacle detection system
    
    Returns:
        LaunchDescription: Complete launch configuration
    """
    
    # =============== Package Configuration ===============
    pkg_share = FindPackageShare('obstacle_detection_pkg')
    
    # Read config file to auto-detect map name (runtime resolution)
    # Try source directory first (development), then install directory
    src_config_file = os.path.join(os.getcwd(), 'src', 'obstacle_detection_pkg', 'config', 'obstacle_detection_params.yaml')
    config_file = src_config_file if os.path.exists(src_config_file) else os.path.join(
        get_package_share_directory('obstacle_detection_pkg'),
        'config',
        'obstacle_detection_params.yaml'
    )
    # Extract default map name from config file
    config_dict = yaml.safe_load(open(config_file, 'r'))
    map_name = config_dict['map_server']['ros__parameters']['map']
    
    # =============== Launch Arguments ===============
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='obstacle_detection_params.yaml',
        description='YAML configuration file for obstacle detection parameters'
    )
    
    # =============== File Paths ===============
    config_file_path = PathJoinSubstitution([
        pkg_share, 'config', LaunchConfiguration('config_file')
    ])
    
    # Map file: resolve path dynamically
    # Try source directory first (for development)
    src_maps_path = os.path.join('src', 'obstacle_detection_pkg', 'maps', map_name + '.yaml')
    if os.path.exists(src_maps_path):
        map_file = os.path.abspath(src_maps_path)
    else:
        # Fallback to package share directory
        map_file = os.path.join(get_package_share_directory('obstacle_detection_pkg'), 'maps', map_name + '.yaml')
    
    # =============== ROS2 Nodes Configuration ===============
    
    # Map server node - provides static environment map for obstacle filtering
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='obstacle_map_server',  # Unique name to avoid conflicts with simulation map server
        parameters=[{
            'yaml_filename': map_file,  # Map file auto-resolved from config
            'use_sim_time': False,  # Use system time instead of simulation time
            'topic_name': '/obstacle_map'  # Publish map on unique topic to avoid conflicts
        }],
        output='screen'
    )
    
    # Lifecycle manager - controls map server lifecycle (configure → activate → deactivate)
    lifecycle_manager_node = TimerAction(
        period=1.0,  # Delay to allow map_server initialization
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_obstacle_map',
                parameters=[{
                    'node_names': ['obstacle_map_server'],  # Manage obstacle map server
                    'use_sim_time': False,
                    'autostart': True,  # Automatically start managed nodes
                    'bond_timeout': 10.0,  # Extended timeout for robustness
                    'bond_disable_heartbeat_timeout': True  # Disable heartbeat timeout
                }],
                output='screen'
            )
        ]
    )
    
    # Obstacle detector node - main processing node for dynamic obstacle detection
    obstacle_detector_node = TimerAction(
        period=2.5,  # Wait for map server to become fully active
        actions=[
            Node(
                package='obstacle_detection_pkg',
                executable='obstacle_detector_node',
                name='obstacle_detector',
                parameters=[config_file_path],  # Load parameters from YAML file
                output='screen'
            )
        ]
    )
    
    # =============== Launch Description Assembly ===============
    return LaunchDescription([
        config_file_arg,          # Configuration file argument
        map_server_node,          # Static map provider
        lifecycle_manager_node,   # Map server lifecycle controller  
        obstacle_detector_node    # Main obstacle detection processor
    ])
