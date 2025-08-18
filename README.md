# Obstacle Detection Package

A real-time dynamic obstacle detection system for autonomous vehicles using LiDAR data and DBSCAN clustering algorithm.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Topics and Messages](#topics-and-messages)
- [Algorithm Details](#algorithm-details)
- [Parameters Tuning](#parameters-tuning)
- [Troubleshooting](#troubleshooting)

## Overview

This package provides robust real-time obstacle detection capabilities for autonomous vehicles in structured environments. The system processes LiDAR scan data to identify and track dynamic obstacles while filtering out static environment features using pre-loaded maps.

### Key Capabilities
- Real-time Processing: 10Hz obstacle detection with low latency
- Static Filtering: Removes known static obstacles using occupancy grid maps
- Robust Clustering: DBSCAN algorithm groups obstacle points into coherent objects
- Multi-format Output: Publishes obstacles as structured messages and visualization markers
- Configurable Parameters: Extensive parameter tuning for different environments

## Features

### Core Functionality
- LiDAR scan processing and coordinate transformation
- Static obstacle filtering using occupancy grid maps  
- DBSCAN-based dynamic obstacle clustering
- Real-time obstacle position and size estimation
- Distance calculation from robot to obstacles

### Output Options
- Custom ObstacleArray messages with detailed obstacle information
- Updated occupancy grid with dynamic obstacles marked
- RViz visualization markers for debugging and monitoring
- Configurable publishing options to optimize performance

### Robustness Features
- Invalid measurement filtering (NaN, infinite ranges)
- Configurable LiDAR range limits
- Error handling and recovery
- Debug output for system monitoring

## System Architecture

```
LiDAR Sensor --> LaserScan Data --> Obstacle Detector Node --> Published Outputs
Robot Odometry --> Pose Data ----/                          |
Map Server --> Occupancy Grid --/                           |
                                                            v
                                        - Detected Obstacles
                                        - Updated Map
                                        - RViz Markers
```

### Processing Pipeline

1. **Data Acquisition**
   - Receive LiDAR scans, robot pose, and static map data
   - Validate input data quality and completeness

2. **Coordinate Transformation** 
   - Convert polar LiDAR measurements to Cartesian coordinates
   - Transform points from sensor frame to global map frame

3. **Static Obstacle Filtering**
   - Compare LiDAR points against static occupancy grid map
   - Remove points within inflation radius of known static obstacles

4. **Dynamic Obstacle Clustering**
   - Apply DBSCAN algorithm to group nearby obstacle points
   - Filter clusters by minimum size requirements

5. **Result Generation**
   - Calculate obstacle centroids, sizes, and distances
   - Publish structured obstacle messages and visualizations

## Installation

### Prerequisites

- ROS2 Humble or later
- PCL (Point Cloud Library)
- Nav2 packages for map server functionality

```bash
# Install required ROS2 packages
sudo apt update
sudo apt install ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager

# Install PCL development libraries
sudo apt install libpcl-dev
```

### Build Instructions

```bash
# Navigate to your ROS2 workspace
cd ~/your_ros2_workspace

# Clone the repository (if not already present)
git clone <repository_url> src/obstacle_detection_pkg

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select obstacle_detection_pkg

# Source the workspace
source install/setup.bash
```

## Usage

### Quick Start

1. **Launch the complete system:**
```bash
ros2 launch obstacle_detection_pkg obstacle_detection.launch.py
```

2. **Launch with custom configuration:**
```bash
ros2 launch obstacle_detection_pkg obstacle_detection.launch.py config_file:=custom_params.yaml
```

### Standalone Node

```bash
# Run only the obstacle detector node (requires external map server)
ros2 run obstacle_detection_pkg obstacle_detector_node --ros-args --params-file src/obstacle_detection_pkg/config/obstacle_detection_params.yaml
```

### Integration with Simulation

```bash
# First, start your main simulation
ros2 launch your_simulation simulation.launch.py

# Then launch obstacle detection
ros2 launch obstacle_detection_pkg obstacle_detection.launch.py
```

## Configuration

### Main Configuration File: `config/obstacle_detection_params.yaml`

#### Topic Configuration
```yaml
scan_topic: "/scan_noisy"                    # LiDAR input topic
odom_topic: "/ego_racecar/odom"              # Robot pose input  
map_topic: "/obstacle_map"                   # Static map input
obstacles_topic: "/detected_obstacles"       # Obstacle output
```

#### DBSCAN Parameters
```yaml
dbscan_eps: 0.3                              # Cluster distance threshold (meters)
dbscan_min_samples: 5                        # Minimum points per cluster
```

#### Processing Parameters  
```yaml
max_lidar_range: 10.0                        # Maximum detection range (meters)
min_lidar_range: 0.1                         # Minimum detection range (meters)
map_inflation_radius: 0.2                    # Static obstacle safety margin (meters)
update_rate: 10.0                            # Processing frequency (Hz)
```

### Map Configuration

The system supports multiple pre-configured maps in the `maps/` directory:

- **Spielberg_map**: F1 race track configuration  
- **levine**: Multi-floor building environment
- **basement_fixed**: Indoor corridor environment
- **map_1753950572**: SLAM-generated real sensor data

## Topics and Messages

### Subscribed Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/scan_noisy` | `sensor_msgs/LaserScan` | LiDAR range measurements |
| `/ego_racecar/odom` | `nav_msgs/Odometry` | Robot pose and velocity |
| `/obstacle_map` | `nav_msgs/OccupancyGrid` | Static environment map |

### Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/detected_obstacles` | `obstacle_detection_pkg/ObstacleArray` | Detected dynamic obstacles |
| `/updated_map` | `nav_msgs/OccupancyGrid` | Map with dynamic obstacles |
| `/obstacle_markers` | `visualization_msgs/MarkerArray` | RViz visualization markers |

### Custom Messages

#### ObstacleArray
```yaml
std_msgs/Header header
uint32 total_obstacles
obstacle_detection_pkg/Obstacle[] obstacles
```

#### Obstacle  
```yaml
geometry_msgs/Point position    # Obstacle centroid position
float64 size                   # Obstacle radius/size estimate  
float64 distance              # Distance from robot
uint32 point_count           # Number of LiDAR points in cluster
```

## Algorithm Details

### DBSCAN Clustering

The system uses Density-Based Spatial Clustering of Applications with Noise (DBSCAN) to group LiDAR points into obstacle clusters:

**Algorithm Parameters:**
- **Epsilon**: Maximum distance between points in the same cluster
- **MinPts**: Minimum number of points required to form a cluster

**Advantages:**
- Handles arbitrary cluster shapes
- Automatically determines number of clusters  
- Robust to noise and outliers
- No prior knowledge of cluster count required

### Static Obstacle Filtering

Points are filtered against the static occupancy grid map:

1. **Map Lookup**: Convert global coordinates to map indices
2. **Inflation Check**: Verify points are outside inflation radius of static obstacles
3. **Occupancy Threshold**: Remove points near cells with occupancy > 50%

## Parameters Tuning

### Performance vs Accuracy Trade-offs

#### For High Accuracy (Lower Speed):
```yaml
dbscan_eps: 0.2                              # Smaller clusters, more precision
dbscan_min_samples: 8                        # Stricter cluster requirements  
update_rate: 5.0                             # Lower frequency, more processing time
```

#### For High Speed (Lower Accuracy):
```yaml
dbscan_eps: 0.5                              # Larger clusters, less precision
dbscan_min_samples: 3                        # Relaxed cluster requirements
update_rate: 20.0                            # Higher frequency, less processing time
max_lidar_range: 5.0                         # Reduced detection range
```

### Environment-Specific Tuning

#### Indoor Environments:
```yaml
map_inflation_radius: 0.1                    # Smaller inflation for narrow spaces
max_lidar_range: 5.0                         # Shorter range for indoor use
```

#### Outdoor/Race Track:
```yaml  
map_inflation_radius: 0.3                    # Larger safety margins
max_lidar_range: 15.0                        # Extended range for high speeds
```

## Troubleshooting

### Common Issues

#### Issue: "Waiting for data - Map: MISSING"
**Cause**: Map server not running or publishing on wrong topic
**Solution**: 
```bash
# Check if map server is running
ros2 node list | grep map_server

# Check map topic
ros2 topic list | grep map
ros2 topic echo /obstacle_map --once
```

#### Issue: "No obstacles detected" despite visible objects
**Cause**: DBSCAN parameters too strict or LiDAR range limits too restrictive  
**Solution**: Adjust parameters in config file:
```yaml
dbscan_eps: 0.5                              # Increase cluster distance
dbscan_min_samples: 3                        # Reduce minimum cluster size
max_lidar_range: 15.0                        # Increase detection range
```

#### Issue: Too many false positive obstacles
**Cause**: DBSCAN parameters too permissive or insufficient static filtering
**Solution**:
```yaml
dbscan_min_samples: 8                        # Increase minimum cluster size
map_inflation_radius: 0.3                    # Increase static obstacle margin
```

### Debug Tools

#### Visualization in RViz:
```bash
# Launch RViz with obstacle markers
ros2 run rviz2 rviz2

# Add MarkerArray display for /obstacle_markers topic
```

#### Topic Monitoring:
```bash
# Monitor obstacle detection output
ros2 topic echo /detected_obstacles

# Check processing frequency
ros2 topic hz /detected_obstacles

# Monitor input data
ros2 topic hz /scan_noisy
ros2 topic hz /ego_racecar/odom
```

### Performance Monitoring

#### CPU Usage:
```bash
# Monitor node CPU usage
top -p $(pgrep -f obstacle_detector_node)
```

## License

This project is licensed under the MIT License.

## Acknowledgments

- ROS2 community for navigation and lifecycle management packages
- PCL library for point cloud processing capabilities
- F1TENTH community for autonomous racing simulation environments

---

**Author**: Jisang Yun  
**Version**: 1.0.0  
**Last Updated**: 2025-01-18