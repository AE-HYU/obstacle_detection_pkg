# Obstacle Detection Package

Real-time dynamic obstacle detection using LiDAR data and DBSCAN clustering for autonomous vehicles.

## Quick Start

```bash
# Build
colcon build --packages-select obstacle_detection_pkg
source install/setup.bash

# Launch complete system (with map server)
ros2 launch obstacle_detection_pkg obstacle_detection.launch.py

# Custom configuration
ros2 launch obstacle_detection_pkg obstacle_detection.launch.py config_file:=custom_params.yaml
```

## Topics

### Input
- `/scan_noisy` - LiDAR range measurements (`sensor_msgs/LaserScan`)
- `/ego_racecar/odom` - Robot pose and velocity (`nav_msgs/Odometry`)
- `/obstacle_map` - Static environment map (`nav_msgs/OccupancyGrid`)

### Output
- `/detected_obstacles` - Dynamic obstacles (`obstacle_detection_pkg/ObstacleArray`)
- `/updated_map` - Map with dynamic obstacles marked (`nav_msgs/OccupancyGrid`)
- `/obstacle_markers` - RViz visualization (`visualization_msgs/MarkerArray`)

## Key Configuration

Edit `config/obstacle_detection_params.yaml`:

```yaml
# DBSCAN clustering
dbscan_eps: 0.3                    # Cluster distance threshold (m)
dbscan_min_samples: 5              # Minimum points per cluster

# Detection range
max_lidar_range: 10.0              # Maximum detection distance (m)
min_lidar_range: 0.1               # Minimum detection distance (m)

# Static filtering
map_inflation_radius: 0.2          # Safety margin from static obstacles (m)

# Performance
update_rate: 10.0                  # Processing frequency (Hz)
```

## Algorithm

1. **Data Processing**: Convert LiDAR scans to global coordinates
2. **Static Filtering**: Remove known static obstacles using occupancy map
3. **DBSCAN Clustering**: Group nearby points into obstacle clusters
4. **Result Generation**: Calculate obstacle positions, sizes, and distances

## Custom Messages

### ObstacleArray
```yaml
std_msgs/Header header
uint32 total_obstacles
obstacle_detection_pkg/Obstacle[] obstacles
```

### Obstacle
```yaml
geometry_msgs/Point position    # Obstacle centroid
float64 size                   # Obstacle radius estimate
float64 distance              # Distance from robot
uint32 point_count           # Number of LiDAR points
```

## Tuning Guidelines

### High Accuracy (slower)
```yaml
dbscan_eps: 0.2                    # Smaller clusters
dbscan_min_samples: 8              # Stricter requirements
update_rate: 5.0                   # Lower frequency
```

### High Speed (less accurate)
```yaml
dbscan_eps: 0.5                    # Larger clusters
dbscan_min_samples: 3              # Relaxed requirements
update_rate: 20.0                  # Higher frequency
max_lidar_range: 5.0               # Reduced range
```

## Available Maps

Pre-configured maps in `maps/` directory:
- `Spielberg_map` - F1 race track
- `levine` - Multi-floor building
- `basement_fixed` - Indoor corridors
- `map_1753950572` - Real sensor data

## Prerequisites

- LiDAR sensor publishing `/scan` data
- Robot odometry available
- Map server providing static environment map
- PCL library installed