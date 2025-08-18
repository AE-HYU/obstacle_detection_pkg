import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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
    
    # Map server node for obstacle detection
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='obstacle_map_server',  # Different name to avoid conflict
        parameters=[{
            'yaml_filename': PathJoinSubstitution([
                pkg_share, 'maps', 'Spielberg_map.yaml'
            ]),
            'use_sim_time': False,
            'topic_name': '/obstacle_map'  # Different topic name
        }],
        output='screen'
    )
    
    # Lifecycle manager for obstacle map server
    lifecycle_manager_node = TimerAction(
        period=1.0,  # Wait for map_server initialization
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_obstacle_map',
                parameters=[{
                    'node_names': ['obstacle_map_server'],
                    'use_sim_time': False,
                    'autostart': True,
                    'bond_timeout': 10.0,
                    'bond_disable_heartbeat_timeout': True
                }],
                output='screen'
            )
        ]
    )
    
    # Obstacle detection node - wait for obstacle map server
    obstacle_detector_node = TimerAction(
        period=2.5,  # Wait for lifecycle_manager to activate map_server
        actions=[
            Node(
                package='obstacle_detection_pkg',
                executable='obstacle_detector_node',
                name='obstacle_detector',
                parameters=[config_file_path],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        map_server_node,
        lifecycle_manager_node,
        obstacle_detector_node
    ])
