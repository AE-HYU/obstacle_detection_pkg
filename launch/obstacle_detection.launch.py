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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Generate launch description for obstacle detection system
    
    Returns:
        LaunchDescription: Complete launch configuration
    """
    
    # =============== Package Configuration ===============
    pkg_share = FindPackageShare('obstacle_detection_pkg')
    
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
    
    # =============== ROS2 Nodes Configuration ===============
    
    # Map server node - provides static environment map for obstacle filtering
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='obstacle_map_server',  # Unique name to avoid conflicts with simulation map server
        parameters=[{
            'yaml_filename': PathJoinSubstitution([
                pkg_share, 'maps', 'Spielberg_map.yaml'  # Static map file for obstacle detection
            ]),
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
