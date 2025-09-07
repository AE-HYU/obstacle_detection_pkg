/**
 * @file obstacle_detector.cpp
 * @brief Implementation of real-time obstacle detection using LiDAR and DBSCAN clustering
 * 
 * This implementation provides:
 * - LiDAR scan processing and coordinate transformation
 * - Static obstacle filtering using occupancy grid maps
 * - DBSCAN clustering algorithm for dynamic obstacle detection  
 * - ROS2 message publishing for detected obstacles and visualization
 */

#include "obstacle_detection_pkg/obstacle_detector.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>

// Helper function for angle normalization (replacing angles/angles.h dependency)
namespace
{
    double shortest_angular_distance(double from, double to)
    {
        double diff = to - from;
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;
        return diff;
    }
}

/**
 * @brief Constructor - Initialize obstacle detection node with parameters and ROS2 components
 * 
 * Sets up all ROS2 subscribers, publishers, parameters, and starts the main processing timer.
 * The node waits for map, pose, and scan data before beginning obstacle detection.
 */
ObstacleDetector::ObstacleDetector() : Node("obstacle_detector"),
    current_robot_velocity_(0.0), map_received_(false), pose_received_(false), scan_received_(false)
{
    // =============== Parameter Declaration ===============
    // ROS2 topic configuration
    this->declare_parameter("scan_topic", "/scan");
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("map_topic", "/map");
    this->declare_parameter("obstacles_topic", "/detected_obstacles");
    this->declare_parameter("updated_map_topic", "/updated_map");
    this->declare_parameter("markers_topic", "/obstacle_markers");
    
    // DBSCAN algorithm parameters
    this->declare_parameter("dbscan_eps", 0.3);
    this->declare_parameter("dbscan_min_samples", 5);
    
    // Obstacle detection parameters
    this->declare_parameter("obstacle_threshold", 0.1);
    this->declare_parameter("map_inflation_radius", 0.2);
    
    // LiDAR processing parameters
    this->declare_parameter("max_lidar_range", 10.0);
    this->declare_parameter("min_lidar_range", 0.1);
    
    // Processing and publishing parameters
    this->declare_parameter("update_rate", 10.0);
    this->declare_parameter("publish_visualization", true);
    this->declare_parameter("publish_updated_map", true);
    
    // High-speed filtering parameters
    this->declare_parameter("velocity_adaptive_filtering", true);
    this->declare_parameter("max_obstacle_persistence_frames", 3);
    this->declare_parameter("min_obstacle_movement_threshold", 0.3);
    
    // Frame ID parameters
    this->declare_parameter("global_frame_id", "map");
    this->declare_parameter("robot_frame_id", "base_link");
    
    // =============== Parameter Retrieval ===============
    // Topic names
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    map_topic_ = this->get_parameter("map_topic").as_string();
    obstacles_topic_ = this->get_parameter("obstacles_topic").as_string();
    updated_map_topic_ = this->get_parameter("updated_map_topic").as_string();
    markers_topic_ = this->get_parameter("markers_topic").as_string();
    
    // Algorithm parameters
    dbscan_eps_ = this->get_parameter("dbscan_eps").as_double();
    dbscan_min_samples_ = this->get_parameter("dbscan_min_samples").as_int();
    obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_double();
    map_inflation_radius_ = this->get_parameter("map_inflation_radius").as_double();
    max_lidar_range_ = this->get_parameter("max_lidar_range").as_double();
    min_lidar_range_ = this->get_parameter("min_lidar_range").as_double();
    update_rate_ = this->get_parameter("update_rate").as_double();
    
    // Publishing options
    publish_visualization_ = this->get_parameter("publish_visualization").as_bool();
    publish_updated_map_ = this->get_parameter("publish_updated_map").as_bool();
    
    // High-speed filtering options
    velocity_adaptive_filtering_ = this->get_parameter("velocity_adaptive_filtering").as_bool();
    max_obstacle_persistence_frames_ = this->get_parameter("max_obstacle_persistence_frames").as_int();
    min_obstacle_movement_threshold_ = this->get_parameter("min_obstacle_movement_threshold").as_double();
    
    // Frame IDs
    global_frame_id_ = this->get_parameter("global_frame_id").as_string();
    robot_frame_id_ = this->get_parameter("robot_frame_id").as_string();
    
    // =============== Transform System Initialization ===============
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // =============== ROS2 Subscribers Setup ===============
    // LiDAR scan subscriber
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, 10, std::bind(&ObstacleDetector::laserScanCallback, this, std::placeholders::_1));
    
    // Robot pose/odometry subscriber
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&ObstacleDetector::poseCallback, this, std::placeholders::_1));
    
    // Static map subscriber with proper QoS for latched topics
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    map_qos.transient_local();  // Receive messages published before subscription
    map_qos.reliable();         // Ensure reliable delivery
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, map_qos, std::bind(&ObstacleDetector::mapCallback, this, std::placeholders::_1));
    
    // =============== ROS2 Publishers Setup ===============
    // Detected obstacles publisher
    obstacles_pub_ = this->create_publisher<obstacle_detection_pkg::msg::ObstacleArray>(
        obstacles_topic_, 10);
    
    // Updated occupancy grid publisher
    updated_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        updated_map_topic_, 10);
    
    // Visualization markers publisher for RViz
    visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        markers_topic_, 10);
    
    // =============== Processing Timer Setup ===============
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_));
    timer_ = this->create_wall_timer(timer_period, 
        std::bind(&ObstacleDetector::processObstacleDetection, this));
    
    RCLCPP_INFO(this->get_logger(), "Obstacle detector initialized - waiting for map, pose, and scan data");
}

// =============== ROS2 Callback Functions ===============

/**
 * @brief Store latest LiDAR scan data and mark as received
 * 
 * This callback is triggered whenever new LiDAR data arrives. The scan data
 * contains range measurements that will be processed in the main detection loop.
 */
void ObstacleDetector::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    latest_scan_ = msg;
    scan_received_ = true;
}

/**
 * @brief Store latest robot pose/odometry data and mark as received
 * 
 * Robot pose is essential for transforming LiDAR points from sensor frame 
 * to the global map frame for obstacle detection processing.
 */
void ObstacleDetector::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    latest_pose_ = msg;
    pose_received_ = true;
}

/**
 * @brief Store static map data and mark as received
 * 
 * The static map is used to filter out points that correspond to known
 * static obstacles, allowing detection of only dynamic obstacles.
 */
void ObstacleDetector::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    static_map_ = msg;
    map_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Static map received: %dx%d, resolution: %.3f", 
                msg->info.width, msg->info.height, msg->info.resolution);
}

// =============== Main Obstacle Detection Processing ===============

/**
 * @brief Main processing loop for obstacle detection (called periodically by timer)
 * 
 * This function coordinates the complete obstacle detection pipeline:
 * 1. Checks that all required data (map, pose, scan) is available
 * 2. Converts LiDAR scan to 2D points in laser frame  
 * 3. Transforms points to global coordinate frame
 * 4. Filters out static map obstacles
 * 5. Clusters remaining points using DBSCAN algorithm
 * 6. Publishes detected obstacles and visualization data
 */
void ObstacleDetector::processObstacleDetection()
{
    // Wait for all required data to be available
    if (!map_received_ || !pose_received_ || !scan_received_) {
        static int debug_counter = 0;
        if (debug_counter % 50 == 0) { // Print status every 5 seconds at 10Hz
            RCLCPP_INFO(this->get_logger(), "Waiting for data - Map: %s, Pose: %s, Scan: %s", 
                        map_received_ ? "OK" : "MISSING",
                        pose_received_ ? "OK" : "MISSING", 
                        scan_received_ ? "OK" : "MISSING");
        }
        debug_counter++;
        return;
    }
    
    try {
        // Step 0: Update robot velocity for adaptive filtering
        updateRobotVelocity();
        
        // Step 1: Convert LiDAR scan to 2D points in laser coordinate frame
        auto points = laserScanToPoints(latest_scan_);
        
        // Step 2: Transform points from laser frame to global map frame
        auto global_points = transformPointsToGlobal(points);
        
        // Step 3: Filter out points corresponding to static map obstacles with velocity compensation
        auto obstacle_points = filterMapObstacles(global_points);
        
        // Step 3.5: Apply temporal consistency filtering for additional robustness
        obstacle_points = applyTemporalFiltering(obstacle_points);
        
        // Step 3.7: Apply velocity-adaptive filtering for high-speed motion
        if (velocity_adaptive_filtering_) {
            obstacle_points = applyVelocityAdaptiveFiltering(obstacle_points);
        }
        
        // Handle case where no dynamic obstacles are detected
        if (obstacle_points.empty()) {
            auto empty_msg = obstacle_detection_pkg::msg::ObstacleArray();
            empty_msg.header.stamp = this->now();
            empty_msg.header.frame_id = global_frame_id_;
            empty_msg.total_obstacles = 0;
            obstacles_pub_->publish(empty_msg);
            return;
        }
        
        // Step 4: Cluster obstacle points using DBSCAN algorithm
        auto clusters = dbscanClustering(obstacle_points);
        
        // Step 5: Create and publish obstacle detection results
        auto obstacle_msg = createObstacleMessage(clusters);
        obstacles_pub_->publish(obstacle_msg);
        
        // Step 6: Publish optional outputs if enabled
        if (publish_updated_map_) {
            auto updated_map = updateOccupancyGrid(clusters);
            updated_map_pub_->publish(updated_map);
        }
        
        if (publish_visualization_) {
            auto markers = createVisualizationMarkers(clusters);
            visualization_pub_->publish(markers);
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Successfully detected %zu obstacle clusters", clusters.size());
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Error in obstacle detection pipeline: %s", e.what());
    }
}

// =============== LiDAR Data Processing Functions ===============

/**
 * @brief Convert polar LiDAR scan data to Cartesian 2D points
 * 
 * Transforms range and angle measurements from LiDAR into 2D Cartesian coordinates
 * in the laser sensor frame. Invalid measurements (NaN, inf, out of range) are filtered out.
 */
std::vector<pcl::PointXY> ObstacleDetector::laserScanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr& scan)
{
    std::vector<pcl::PointXY> points;
    
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float range = scan->ranges[i];
        
        // Filter invalid ranges
        if (range < min_lidar_range_ || range > max_lidar_range_ || 
            std::isnan(range) || std::isinf(range)) {
            continue;
        }
        
        // Convert to Cartesian coordinates in laser frame
        float angle = scan->angle_min + i * scan->angle_increment;
        pcl::PointXY point;
        point.x = range * cos(angle);
        point.y = range * sin(angle);
        points.push_back(point);
    }
    
    return points;
}

std::vector<pcl::PointXY> ObstacleDetector::transformPointsToGlobal(const std::vector<pcl::PointXY>& points)
{
    std::vector<pcl::PointXY> global_points;
    
    if (!latest_pose_) {
        return global_points;
    }
    
    // Get robot pose
    double robot_x = latest_pose_->pose.pose.position.x;
    double robot_y = latest_pose_->pose.pose.position.y;
    
    // Extract yaw from quaternion
    tf2::Quaternion q;
    tf2::fromMsg(latest_pose_->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // Transform each point
    for (const auto& point : points) {
        pcl::PointXY global_point;
        global_point.x = robot_x + point.x * cos(yaw) - point.y * sin(yaw);
        global_point.y = robot_y + point.x * sin(yaw) + point.y * cos(yaw);
        global_points.push_back(global_point);
    }
    
    return global_points;
}

std::vector<pcl::PointXY> ObstacleDetector::filterMapObstacles(const std::vector<pcl::PointXY>& points)
{
    std::vector<pcl::PointXY> filtered_points;
    
    for (const auto& point : points) {
        if (isPointInMap(point.x, point.y)) {
            // Use velocity compensation if enabled, otherwise use standard filtering
            bool is_static_obstacle = velocity_adaptive_filtering_ ? 
                isMapObstacleWithVelocityCompensation(point.x, point.y) : 
                isMapObstacle(point.x, point.y);
                
            if (!is_static_obstacle) {
                filtered_points.push_back(point);
            }
        }
    }
    
    return filtered_points;
}

std::vector<std::vector<pcl::PointXY>> ObstacleDetector::dbscanClustering(const std::vector<pcl::PointXY>& points)
{
    std::vector<std::vector<pcl::PointXY>> clusters;
    
    if (points.empty()) {
        return clusters;
    }
    
    // DBSCAN constants
    const int UNCLASSIFIED = -1;
    const int NOISE = -2;
    
    std::vector<int> cluster_id(points.size(), UNCLASSIFIED);
    int current_cluster = 0;
    
    // Main DBSCAN loop
    for (size_t i = 0; i < points.size(); ++i) {
        if (cluster_id[i] != UNCLASSIFIED) continue;
        
        // Find neighbors of current point
        std::vector<size_t> neighbors = getNeighbors(points, i);
        
        if (static_cast<int>(neighbors.size()) < dbscan_min_samples_) {
            cluster_id[i] = NOISE;
        } else {
            // Expand cluster
            expandCluster(points, cluster_id, i, neighbors, current_cluster);
            current_cluster++;
        }
    }
    
    // Group points by cluster
    clusters.resize(current_cluster);
    for (size_t i = 0; i < points.size(); ++i) {
        if (cluster_id[i] >= 0) {
            clusters[cluster_id[i]].push_back(points[i]);
        }
    }
    
    return clusters;
}

std::vector<size_t> ObstacleDetector::getNeighbors(const std::vector<pcl::PointXY>& points, size_t point_idx)
{
    std::vector<size_t> neighbors;
    const auto& center = points[point_idx];
    
    for (size_t i = 0; i < points.size(); ++i) {
        if (i == point_idx) continue;
        
        // Use squared distance (faster)
        double dist_sq = pow(center.x - points[i].x, 2) + pow(center.y - points[i].y, 2);
        if (dist_sq <= dbscan_eps_ * dbscan_eps_) {
            neighbors.push_back(i);
        }
    }
    
    return neighbors;
}

void ObstacleDetector::expandCluster(const std::vector<pcl::PointXY>& points, 
                                    std::vector<int>& cluster_id,
                                    size_t point_idx,
                                    std::vector<size_t>& seed_set,
                                    int cluster_num)
{
    const int UNCLASSIFIED = -1;
    const int NOISE = -2;
    
    cluster_id[point_idx] = cluster_num;
    
    // Process seed set (use index to handle dynamic growth)
    for (size_t i = 0; i < seed_set.size(); ++i) {
        size_t current_point = seed_set[i];
        
        if (cluster_id[current_point] == NOISE) {
            cluster_id[current_point] = cluster_num;
        }
        
        if (cluster_id[current_point] != UNCLASSIFIED) continue;
        
        cluster_id[current_point] = cluster_num;
        
        // Get neighbors of current point
        std::vector<size_t> neighbors = getNeighbors(points, current_point);
        
        // If core point, add unvisited neighbors to seed set
        if (static_cast<int>(neighbors.size()) >= dbscan_min_samples_) {
            for (size_t neighbor : neighbors) {
                if (cluster_id[neighbor] == UNCLASSIFIED) {
                    seed_set.push_back(neighbor);
                }
            }
        }
    }
}

bool ObstacleDetector::isPointInMap(double x, double y) const
{
    if (!static_map_) {
        return false;
    }
    
    // Convert world coordinates to map indices
    int mx = static_cast<int>((x - static_map_->info.origin.position.x) / static_map_->info.resolution);
    int my = static_cast<int>((y - static_map_->info.origin.position.y) / static_map_->info.resolution);
    
    return (mx >= 0 && mx < static_cast<int>(static_map_->info.width) && 
            my >= 0 && my < static_cast<int>(static_map_->info.height));
}

bool ObstacleDetector::isMapObstacle(double x, double y) const
{
    if (!static_map_) {
        return false;
    }
    
    // Enhanced multi-layer filtering for robust static obstacle detection
    
    // Layer 1: Direct cell check with conservative threshold
    int mx = static_cast<int>((x - static_map_->info.origin.position.x) / static_map_->info.resolution);
    int my = static_cast<int>((y - static_map_->info.origin.position.y) / static_map_->info.resolution);
    
    // Bounds check
    if (mx < 0 || mx >= static_cast<int>(static_map_->info.width) || 
        my < 0 || my >= static_cast<int>(static_map_->info.height)) {
        return true; // Consider out-of-bounds as obstacle
    }
    
    int center_index = my * static_map_->info.width + mx;
    
    // Very conservative threshold - even uncertain cells are treated as obstacles
    if (static_map_->data[center_index] > 10) {
        return true;
    }
    
    // Layer 2: Inflated area check - examine larger radius for safety
    double enhanced_inflation_radius = map_inflation_radius_ * 1.5; // 50% larger safety margin
    int inflation_cells = static_cast<int>(enhanced_inflation_radius / static_map_->info.resolution);
    
    // Count occupied cells in inflation area
    int occupied_count = 0;
    int total_checked = 0;
    
    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
            // Skip center (already checked)
            if (dx == 0 && dy == 0) continue;
            
            // Check if point is within circular inflation radius
            double distance = sqrt(dx*dx + dy*dy) * static_map_->info.resolution;
            if (distance > enhanced_inflation_radius) continue;
            
            int check_mx = mx + dx;
            int check_my = my + dy;
            
            // Bounds check
            if (check_mx < 0 || check_mx >= static_cast<int>(static_map_->info.width) || 
                check_my < 0 || check_my >= static_cast<int>(static_map_->info.height)) {
                occupied_count++; // Out-of-bounds considered as obstacle
                total_checked++;
                continue;
            }
            
            int check_index = check_my * static_map_->info.width + check_mx;
            total_checked++;
            
            // More conservative threshold for surrounding area
            if (static_map_->data[check_index] > 15) {
                occupied_count++;
            }
        }
    }
    
    // Layer 3: Density-based filtering
    // If more than 20% of surrounding cells are occupied, consider as static obstacle
    if (total_checked > 0) {
        double obstacle_density = static_cast<double>(occupied_count) / total_checked;
        if (obstacle_density > 0.2) {
            return true;
        }
    }
    
    // Layer 4: Extended neighborhood check for map alignment issues
    // Check a larger area to catch map misalignment cases
    int extended_cells = static_cast<int>((map_inflation_radius_ * 2.0) / static_map_->info.resolution);
    int extended_occupied = 0;
    int extended_total = 0;
    
    for (int dx = -extended_cells; dx <= extended_cells; dx += 2) { // Skip every other cell for efficiency
        for (int dy = -extended_cells; dy <= extended_cells; dy += 2) {
            int ext_mx = mx + dx;
            int ext_my = my + dy;
            
            if (ext_mx < 0 || ext_mx >= static_cast<int>(static_map_->info.width) || 
                ext_my < 0 || ext_my >= static_cast<int>(static_map_->info.height)) {
                continue;
            }
            
            int ext_index = ext_my * static_map_->info.width + ext_mx;
            extended_total++;
            
            if (static_map_->data[ext_index] > 25) {
                extended_occupied++;
            }
        }
    }
    
    // If extended area has high obstacle density, likely near static structure
    if (extended_total > 0) {
        double extended_density = static_cast<double>(extended_occupied) / extended_total;
        if (extended_density > 0.15) {
            return true;
        }
    }
    
    return false;
}

nav_msgs::msg::OccupancyGrid ObstacleDetector::updateOccupancyGrid(const std::vector<std::vector<pcl::PointXY>>& clusters)
{
    nav_msgs::msg::OccupancyGrid updated_map = *static_map_;
    updated_map.header.stamp = this->now();
    
    // Mark obstacle clusters in the grid
    for (const auto& cluster : clusters) {
        for (const auto& point : cluster) {
            int mx = static_cast<int>((point.x - static_map_->info.origin.position.x) / static_map_->info.resolution);
            int my = static_cast<int>((point.y - static_map_->info.origin.position.y) / static_map_->info.resolution);
            
            if (mx >= 0 && mx < static_cast<int>(static_map_->info.width) && 
                my >= 0 && my < static_cast<int>(static_map_->info.height)) {
                
                int index = my * static_map_->info.width + mx;
                updated_map.data[index] = 100; // Mark as occupied
            }
        }
    }
    
    return updated_map;
}

obstacle_detection_pkg::msg::ObstacleArray ObstacleDetector::createObstacleMessage(const std::vector<std::vector<pcl::PointXY>>& clusters)
{
    obstacle_detection_pkg::msg::ObstacleArray msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = global_frame_id_;
    msg.total_obstacles = clusters.size();
    
    for (const auto& cluster : clusters) {
        obstacle_detection_pkg::msg::Obstacle obstacle;
        
        // Calculate centroid
        double sum_x = 0.0, sum_y = 0.0;
        for (const auto& point : cluster) {
            sum_x += point.x;
            sum_y += point.y;
        }
        
        obstacle.position.x = sum_x / cluster.size();
        obstacle.position.y = sum_y / cluster.size();
        obstacle.position.z = 0.0;
        
        // Calculate approximate size (max distance from centroid)
        double max_dist = 0.0;
        for (const auto& point : cluster) {
            double dist = sqrt(pow(point.x - obstacle.position.x, 2) + 
                              pow(point.y - obstacle.position.y, 2));
            max_dist = std::max(max_dist, dist);
        }
        obstacle.size = max_dist;
        
        // Calculate distance from robot to obstacle
        if (latest_pose_) {
            double robot_x = latest_pose_->pose.pose.position.x;
            double robot_y = latest_pose_->pose.pose.position.y;
            obstacle.distance = sqrt(pow(obstacle.position.x - robot_x, 2) + 
                                   pow(obstacle.position.y - robot_y, 2));
        } else {
            obstacle.distance = 0.0;
        }
        
        obstacle.point_count = cluster.size();
        
        msg.obstacles.push_back(obstacle);
    }
    
    return msg;
}

visualization_msgs::msg::MarkerArray ObstacleDetector::createVisualizationMarkers(const std::vector<std::vector<pcl::PointXY>>& clusters)
{
    visualization_msgs::msg::MarkerArray markers;
    
    // Delete all previous markers
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = global_frame_id_;
    delete_marker.header.stamp = this->now();
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    
    // Create markers for each cluster
    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& cluster = clusters[i];
        
        // Calculate centroid
        double sum_x = 0.0, sum_y = 0.0;
        for (const auto& point : cluster) {
            sum_x += point.x;
            sum_y += point.y;
        }
        double centroid_x = sum_x / cluster.size();
        double centroid_y = sum_y / cluster.size();
        
        // Create sphere marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = global_frame_id_;
        marker.header.stamp = this->now();
        marker.ns = "obstacles";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = centroid_x;
        marker.pose.position.y = centroid_y;
        marker.pose.position.z = 0.5;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        
        marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        
        markers.markers.push_back(marker);
    }
    
    return markers;
}

std::vector<pcl::PointXY> ObstacleDetector::applyTemporalFiltering(const std::vector<pcl::PointXY>& current_points)
{
    std::vector<pcl::PointXY> filtered_points;
    
    // Initialize on first run
    if (previous_obstacle_points_.empty()) {
        previous_obstacle_points_.push_back(current_points);
        last_processing_time_ = this->now();
        // Return all points on first frame (no history to compare against)
        return current_points;
    }
    
    // Time-based consistency check
    rclcpp::Time current_time = this->now();
    double time_delta = (current_time - last_processing_time_).seconds();
    
    // Skip temporal filtering if too much time has passed (robot might have moved significantly)
    if (time_delta > 1.0) {
        previous_obstacle_points_.clear();
        previous_obstacle_points_.push_back(current_points);
        last_processing_time_ = current_time;
        return current_points;
    }
    
    // Temporal consistency filtering
    const double consistency_threshold = 0.8; // Points must be within 80cm to be considered "same location"
    const int min_frame_presence = 2; // Point must appear in at least 2 frames to be considered dynamic
    
    for (const auto& current_point : current_points) {
        int presence_count = 1; // Current frame counts as 1
        
        // Check presence in previous frames
        for (const auto& prev_frame : previous_obstacle_points_) {
            bool found_in_frame = false;
            
            for (const auto& prev_point : prev_frame) {
                double distance = sqrt(pow(current_point.x - prev_point.x, 2) + 
                                     pow(current_point.y - prev_point.y, 2));
                
                if (distance < consistency_threshold) {
                    found_in_frame = true;
                    break;
                }
            }
            
            if (found_in_frame) {
                presence_count++;
            }
        }
        
        // Additional check: reject points that appear too consistently (likely static)
        // If a point appears in ALL previous frames at the same location, it's probably static
        if (presence_count >= min_frame_presence && presence_count < static_cast<int>(previous_obstacle_points_.size() + 1)) {
            filtered_points.push_back(current_point);
        } else if (previous_obstacle_points_.size() < 3) {
            // In early frames, be more permissive
            filtered_points.push_back(current_point);
        }
        // Points that appear in ALL frames are likely static misclassifications, so we filter them out
    }
    
    // Update history (keep only last 5 frames for efficiency)
    previous_obstacle_points_.push_back(current_points);
    if (previous_obstacle_points_.size() > 5) {
        previous_obstacle_points_.erase(previous_obstacle_points_.begin());
    }
    
    last_processing_time_ = current_time;
    
    RCLCPP_DEBUG(this->get_logger(), 
        "Temporal filtering: %zu -> %zu points (removed %zu potentially static points)",
        current_points.size(), filtered_points.size(), 
        current_points.size() - filtered_points.size());
    
    return filtered_points;
}

void ObstacleDetector::updateRobotVelocity()
{
    if (!latest_pose_) {
        current_robot_velocity_ = 0.0;
        return;
    }
    
    // Update position history
    geometry_msgs::msg::Point current_pos = latest_pose_->pose.pose.position;
    previous_robot_positions_.push_back(current_pos);
    
    // Keep only last 5 positions for velocity calculation
    if (previous_robot_positions_.size() > 5) {
        previous_robot_positions_.erase(previous_robot_positions_.begin());
    }
    
    // Calculate velocity from position history
    if (previous_robot_positions_.size() >= 2) {
        auto& prev_pos = previous_robot_positions_[previous_robot_positions_.size() - 2];
        auto& curr_pos = previous_robot_positions_.back();
        
        double distance = sqrt(pow(curr_pos.x - prev_pos.x, 2) + pow(curr_pos.y - prev_pos.y, 2));
        double time_delta = 1.0 / update_rate_; // Approximate time delta
        current_robot_velocity_ = distance / time_delta;
        
        // Smooth velocity using moving average
        static std::vector<double> velocity_history;
        velocity_history.push_back(current_robot_velocity_);
        if (velocity_history.size() > 3) {
            velocity_history.erase(velocity_history.begin());
        }
        
        double sum = 0.0;
        for (double v : velocity_history) {
            sum += v;
        }
        current_robot_velocity_ = sum / velocity_history.size();
    }
}

bool ObstacleDetector::isMapObstacleWithVelocityCompensation(double x, double y) const
{
    if (!static_map_) {
        return false;
    }
    
    // Enhanced multi-layer filtering with velocity-based adaptations
    
    // Velocity-based margin scaling
    double velocity_scale_factor = 1.0 + (current_robot_velocity_ * 0.5); // 50% increase per m/s
    double adaptive_inflation_radius = map_inflation_radius_ * velocity_scale_factor;
    
    // Layer 1: Direct cell check with extra conservative threshold for high-speed
    int mx = static_cast<int>((x - static_map_->info.origin.position.x) / static_map_->info.resolution);
    int my = static_cast<int>((y - static_map_->info.origin.position.y) / static_map_->info.resolution);
    
    if (mx < 0 || mx >= static_cast<int>(static_map_->info.width) || 
        my < 0 || my >= static_cast<int>(static_map_->info.height)) {
        return true;
    }
    
    int center_index = my * static_map_->info.width + mx;
    
    // Ultra-conservative threshold for high-speed scenarios
    int occupancy_threshold = current_robot_velocity_ > 3.0 ? 5 : 10;
    if (static_map_->data[center_index] > occupancy_threshold) {
        return true;
    }
    
    // Layer 2: Expanded inflation area with velocity compensation
    int inflation_cells = static_cast<int>(adaptive_inflation_radius / static_map_->info.resolution);
    int occupied_count = 0;
    int total_checked = 0;
    
    // Check larger area for high-speed motion
    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
            if (dx == 0 && dy == 0) continue;
            
            double distance = sqrt(dx*dx + dy*dy) * static_map_->info.resolution;
            if (distance > adaptive_inflation_radius) continue;
            
            int check_mx = mx + dx;
            int check_my = my + dy;
            
            if (check_mx < 0 || check_mx >= static_cast<int>(static_map_->info.width) || 
                check_my < 0 || check_my >= static_cast<int>(static_map_->info.height)) {
                occupied_count++;
                total_checked++;
                continue;
            }
            
            int check_index = check_my * static_map_->info.width + check_mx;
            total_checked++;
            
            // More conservative threshold for high-speed
            int area_threshold = current_robot_velocity_ > 3.0 ? 8 : 15;
            if (static_map_->data[check_index] > area_threshold) {
                occupied_count++;
            }
        }
    }
    
    // Layer 3: Velocity-adaptive density check
    if (total_checked > 0) {
        double density_threshold = current_robot_velocity_ > 3.0 ? 0.1 : 0.2; // Stricter for high-speed
        double obstacle_density = static_cast<double>(occupied_count) / total_checked;
        if (obstacle_density > density_threshold) {
            return true;
        }
    }
    
    // Layer 4: Extended area check with velocity compensation
    int extended_cells = static_cast<int>((adaptive_inflation_radius * 1.5) / static_map_->info.resolution);
    int extended_occupied = 0;
    int extended_total = 0;
    
    for (int dx = -extended_cells; dx <= extended_cells; dx += 3) { // Larger steps for efficiency
        for (int dy = -extended_cells; dy <= extended_cells; dy += 3) {
            int ext_mx = mx + dx;
            int ext_my = my + dy;
            
            if (ext_mx < 0 || ext_mx >= static_cast<int>(static_map_->info.width) || 
                ext_my < 0 || ext_my >= static_cast<int>(static_map_->info.height)) {
                continue;
            }
            
            int ext_index = ext_my * static_map_->info.width + ext_mx;
            extended_total++;
            
            if (static_map_->data[ext_index] > 25) {
                extended_occupied++;
            }
        }
    }
    
    if (extended_total > 0) {
        double extended_threshold = current_robot_velocity_ > 3.0 ? 0.08 : 0.15;
        double extended_density = static_cast<double>(extended_occupied) / extended_total;
        if (extended_density > extended_threshold) {
            return true;
        }
    }
    
    return false;
}

std::vector<pcl::PointXY> ObstacleDetector::applyVelocityAdaptiveFiltering(const std::vector<pcl::PointXY>& points)
{
    std::vector<pcl::PointXY> filtered_points;
    
    if (!latest_pose_) {
        return points; // No pose data, return as-is
    }
    
    double robot_x = latest_pose_->pose.pose.position.x;
    double robot_y = latest_pose_->pose.pose.position.y;
    
    // High-speed specific filtering
    for (const auto& point : points) {
        bool should_keep = true;
        
        // Filter 1: Distance-based filtering with velocity scaling
        double distance_to_robot = sqrt(pow(point.x - robot_x, 2) + pow(point.y - robot_y, 2));
        double min_distance_threshold = 0.5 + (current_robot_velocity_ * 0.2); // Increase with speed
        
        if (distance_to_robot < min_distance_threshold) {
            should_keep = false; // Too close, likely sensor noise or ground reflection
        }
        
        // Filter 2: Velocity-based direction filtering
        if (current_robot_velocity_ > 2.0) { // Only for high-speed motion
            // Calculate direction from robot to obstacle
            double dx = point.x - robot_x;
            double dy = point.y - robot_y;
            
            // Check if obstacle is in unexpected positions for high-speed motion
            // (e.g., directly behind the robot - likely false positive from map misalignment)
            double angle_to_obstacle = atan2(dy, dx);
            
            // Get robot heading from pose
            tf2::Quaternion q;
            tf2::fromMsg(latest_pose_->pose.pose.orientation, q);
            double roll, pitch, robot_yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, robot_yaw);
            
            double relative_angle = shortest_angular_distance(robot_yaw, angle_to_obstacle);
            
            // Filter out obstacles directly behind robot during high-speed motion
            if (std::abs(relative_angle) > 2.5 && distance_to_robot < 3.0) { // Behind robot and close
                should_keep = false;
            }
        }
        
        // Filter 3: Persistence check for stationary "obstacles"
        // Check if this obstacle has been stationary relative to the map for too many frames
        if (should_keep && static_cast<int>(previous_obstacle_points_.size()) >= max_obstacle_persistence_frames_) {
            int persistence_count = 0;
            double persistence_threshold = min_obstacle_movement_threshold_;
            
            for (const auto& prev_frame : previous_obstacle_points_) {
                bool found_stationary = false;
                for (const auto& prev_point : prev_frame) {
                    double movement = sqrt(pow(point.x - prev_point.x, 2) + pow(point.y - prev_point.y, 2));
                    if (movement < persistence_threshold) {
                        found_stationary = true;
                        break;
                    }
                }
                if (found_stationary) {
                    persistence_count++;
                }
            }
            
            // If obstacle has been stationary in most previous frames, likely static misclassification
            if (persistence_count >= max_obstacle_persistence_frames_ - 1) {
                should_keep = false;
            }
        }
        
        if (should_keep) {
            filtered_points.push_back(point);
        }
    }
    
    RCLCPP_DEBUG(this->get_logger(), 
        "Velocity-adaptive filtering (v=%.2f m/s): %zu -> %zu points", 
        current_robot_velocity_, points.size(), filtered_points.size());
    
    return filtered_points;
}