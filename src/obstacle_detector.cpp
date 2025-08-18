#include "obstacle_detection_pkg/obstacle_detector.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>

ObstacleDetector::ObstacleDetector() : Node("obstacle_detector"),
    map_received_(false), pose_received_(false), scan_received_(false)
{
    // Declare parameters
    this->declare_parameter("scan_topic", "/scan");
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("map_topic", "/map");
    this->declare_parameter("obstacles_topic", "/detected_obstacles");
    this->declare_parameter("updated_map_topic", "/updated_map");
    this->declare_parameter("markers_topic", "/obstacle_markers");
    this->declare_parameter("dbscan_eps", 0.3);
    this->declare_parameter("dbscan_min_samples", 5);
    this->declare_parameter("obstacle_threshold", 0.1);
    this->declare_parameter("map_inflation_radius", 0.2);
    this->declare_parameter("max_lidar_range", 10.0);
    this->declare_parameter("min_lidar_range", 0.1);
    this->declare_parameter("update_rate", 10.0);
    this->declare_parameter("publish_visualization", true);
    this->declare_parameter("publish_updated_map", true);
    this->declare_parameter("global_frame_id", "map");
    this->declare_parameter("robot_frame_id", "base_link");
    
    // Get parameters
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    map_topic_ = this->get_parameter("map_topic").as_string();
    obstacles_topic_ = this->get_parameter("obstacles_topic").as_string();
    updated_map_topic_ = this->get_parameter("updated_map_topic").as_string();
    markers_topic_ = this->get_parameter("markers_topic").as_string();
    dbscan_eps_ = this->get_parameter("dbscan_eps").as_double();
    dbscan_min_samples_ = this->get_parameter("dbscan_min_samples").as_int();
    obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_double();
    map_inflation_radius_ = this->get_parameter("map_inflation_radius").as_double();
    max_lidar_range_ = this->get_parameter("max_lidar_range").as_double();
    min_lidar_range_ = this->get_parameter("min_lidar_range").as_double();
    update_rate_ = this->get_parameter("update_rate").as_double();
    publish_visualization_ = this->get_parameter("publish_visualization").as_bool();
    publish_updated_map_ = this->get_parameter("publish_updated_map").as_bool();
    global_frame_id_ = this->get_parameter("global_frame_id").as_string();
    robot_frame_id_ = this->get_parameter("robot_frame_id").as_string();
    
    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Create subscribers
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, 10, std::bind(&ObstacleDetector::laserScanCallback, this, std::placeholders::_1));
    
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&ObstacleDetector::poseCallback, this, std::placeholders::_1));
    
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, 10, std::bind(&ObstacleDetector::mapCallback, this, std::placeholders::_1));
    
    // Create publishers
    obstacles_pub_ = this->create_publisher<obstacle_detection_pkg::msg::ObstacleArray>(
        obstacles_topic_, 10);
    
    updated_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        updated_map_topic_, 10);
    
    visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        markers_topic_, 10);
    
    // Create timer for processing
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_));
    timer_ = this->create_wall_timer(timer_period, 
        std::bind(&ObstacleDetector::processObstacleDetection, this));
    
    RCLCPP_INFO(this->get_logger(), "Obstacle detector initialized");
}

void ObstacleDetector::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    latest_scan_ = msg;
    scan_received_ = true;
}

void ObstacleDetector::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    latest_pose_ = msg;
    pose_received_ = true;
}

void ObstacleDetector::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    static_map_ = msg;
    map_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Static map received: %dx%d, resolution: %.3f", 
                msg->info.width, msg->info.height, msg->info.resolution);
}

void ObstacleDetector::processObstacleDetection()
{
    if (!map_received_ || !pose_received_ || !scan_received_) {
        return;
    }
    
    try {
        // Convert laser scan to points
        auto points = laserScanToPoints(latest_scan_);
        
        // Transform points to global frame
        auto global_points = transformPointsToGlobal(points);
        
        // Filter out static map obstacles
        auto obstacle_points = filterMapObstacles(global_points);
        
        if (obstacle_points.empty()) {
            // Publish empty obstacle array
            auto empty_msg = obstacle_detection_pkg::msg::ObstacleArray();
            empty_msg.header.stamp = this->now();
            empty_msg.header.frame_id = global_frame_id_;
            empty_msg.total_obstacles = 0;
            obstacles_pub_->publish(empty_msg);
            return;
        }
        
        // Cluster obstacle points using DBSCAN
        auto clusters = dbscanClustering(obstacle_points);
        
        // Publish results
        auto obstacle_msg = createObstacleMessage(clusters);
        obstacles_pub_->publish(obstacle_msg);
        
        if (publish_updated_map_) {
            auto updated_map = updateOccupancyGrid(clusters);
            updated_map_pub_->publish(updated_map);
        }
        
        if (publish_visualization_) {
            auto markers = createVisualizationMarkers(clusters);
            visualization_pub_->publish(markers);
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Detected %zu obstacle clusters", clusters.size());
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Error in obstacle detection: %s", e.what());
    }
}

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
        if (isPointInMap(point.x, point.y) && !isMapObstacle(point.x, point.y)) {
            filtered_points.push_back(point);
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
    
    // Check in inflated area around point
    int inflation_cells = static_cast<int>(map_inflation_radius_ / static_map_->info.resolution);
    
    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
            double check_x = x + dx * static_map_->info.resolution;
            double check_y = y + dy * static_map_->info.resolution;
            
            int mx = static_cast<int>((check_x - static_map_->info.origin.position.x) / static_map_->info.resolution);
            int my = static_cast<int>((check_y - static_map_->info.origin.position.y) / static_map_->info.resolution);
            
            if (mx >= 0 && mx < static_cast<int>(static_map_->info.width) && 
                my >= 0 && my < static_cast<int>(static_map_->info.height)) {
                
                int index = my * static_map_->info.width + mx;
                if (static_map_->data[index] > 50) { // Occupied cell
                    return true;
                }
            }
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