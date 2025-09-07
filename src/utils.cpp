/**
 * @file utils.cpp
 * @brief Implementation of utility functions for obstacle detection
 * 
 * Contains implementations for:
 * - DBSCAN clustering algorithm
 * - Map-based obstacle filtering with velocity compensation
 * - Temporal consistency filtering
 * - High-speed motion adaptive filtering
 * - Message creation utilities
 * 
 * @author Jisang Yun
 * @license MIT
 */

#include "obstacle_detection_pkg/utils.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace obstacle_detection_utils
{

// =============== Data Processing Utilities ===============

std::vector<pcl::PointXY> laser_scan_to_points(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan,
    double min_range, 
    double max_range)
{
    std::vector<pcl::PointXY> points;
    
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float range = scan->ranges[i];
        float angle = scan->angle_min + i * scan->angle_increment;
        
        // Filter invalid measurements
        if (std::isnan(range) || std::isinf(range) || 
            range < min_range || range > max_range) {
            continue;
        }
        
        // Convert polar to Cartesian coordinates
        pcl::PointXY point;
        point.x = range * cos(angle);
        point.y = range * sin(angle);
        points.push_back(point);
    }
    
    return points;
}

std::vector<pcl::PointXY> transform_points_to_global(
    const std::vector<pcl::PointXY>& points,
    const nav_msgs::msg::Odometry::SharedPtr& robot_pose)
{
    std::vector<pcl::PointXY> global_points;
    
    if (!robot_pose) {
        return global_points;
    }
    
    // Extract robot position and orientation
    double robot_x = robot_pose->pose.pose.position.x;
    double robot_y = robot_pose->pose.pose.position.y;
    double yaw = extract_yaw_from_pose(robot_pose);
    
    // Transform each point to global frame
    for (const auto& point : points) {
        pcl::PointXY global_point;
        
        // Apply 2D rotation and translation
        global_point.x = robot_x + point.x * cos(yaw) - point.y * sin(yaw);
        global_point.y = robot_y + point.x * sin(yaw) + point.y * cos(yaw);
        global_points.push_back(global_point);
    }
    
    return global_points;
}

// =============== Map-based Filtering Utilities ===============

bool is_point_in_map(double x, double y, const nav_msgs::msg::OccupancyGrid::SharedPtr& map)
{
    if (!map) {
        return false;
    }
    
    // Convert world coordinates to map indices
    int mx = static_cast<int>((x - map->info.origin.position.x) / map->info.resolution);
    int my = static_cast<int>((y - map->info.origin.position.y) / map->info.resolution);
    
    return (mx >= 0 && mx < static_cast<int>(map->info.width) && 
            my >= 0 && my < static_cast<int>(map->info.height));
}

bool is_map_obstacle(double x, double y, 
                     const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
                     double inflation_radius)
{
    if (!map) {
        return false;
    }
    
    // Enhanced multi-layer filtering for robust static obstacle detection
    
    // Layer 1: Direct cell check with conservative threshold
    int mx = static_cast<int>((x - map->info.origin.position.x) / map->info.resolution);
    int my = static_cast<int>((y - map->info.origin.position.y) / map->info.resolution);
    
    // Bounds check
    if (mx < 0 || mx >= static_cast<int>(map->info.width) || 
        my < 0 || my >= static_cast<int>(map->info.height)) {
        return true; // Consider out-of-bounds as obstacle
    }
    
    int center_index = my * map->info.width + mx;
    
    // Very conservative threshold - even uncertain cells are treated as obstacles
    if (map->data[center_index] > 10) {
        return true;
    }
    
    // Layer 2: Inflated area check - examine larger radius for safety
    double enhanced_inflation_radius = inflation_radius * 1.5; // 50% larger safety margin
    int inflation_cells = static_cast<int>(enhanced_inflation_radius / map->info.resolution);
    
    // Count occupied cells in inflation area
    int occupied_count = 0;
    int total_checked = 0;
    
    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
            // Skip center (already checked)
            if (dx == 0 && dy == 0) continue;
            
            // Check if point is within circular inflation radius
            double distance = sqrt(dx*dx + dy*dy) * map->info.resolution;
            if (distance > enhanced_inflation_radius) continue;
            
            int check_mx = mx + dx;
            int check_my = my + dy;
            
            // Bounds check
            if (check_mx < 0 || check_mx >= static_cast<int>(map->info.width) || 
                check_my < 0 || check_my >= static_cast<int>(map->info.height)) {
                occupied_count++; // Out-of-bounds considered as obstacle
                total_checked++;
                continue;
            }
            
            int check_index = check_my * map->info.width + check_mx;
            total_checked++;
            
            // More conservative threshold for surrounding area
            if (map->data[check_index] > 15) {
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
    int extended_cells = static_cast<int>((inflation_radius * 2.0) / map->info.resolution);
    int extended_occupied = 0;
    int extended_total = 0;
    
    for (int dx = -extended_cells; dx <= extended_cells; dx += 2) { // Skip every other cell for efficiency
        for (int dy = -extended_cells; dy <= extended_cells; dy += 2) {
            int ext_mx = mx + dx;
            int ext_my = my + dy;
            
            if (ext_mx < 0 || ext_mx >= static_cast<int>(map->info.width) || 
                ext_my < 0 || ext_my >= static_cast<int>(map->info.height)) {
                continue;
            }
            
            int ext_index = ext_my * map->info.width + ext_mx;
            extended_total++;
            
            if (map->data[ext_index] > 25) {
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

bool is_map_obstacle_with_velocity_compensation(double x, double y,
                                                const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
                                                double base_inflation_radius,
                                                double robot_velocity)
{
    if (!map) {
        return false;
    }
    
    // Enhanced multi-layer filtering with velocity-based adaptations
    
    // Velocity-based margin scaling
    double velocity_scale_factor = 1.0 + (robot_velocity * 0.5); // 50% increase per m/s
    double adaptive_inflation_radius = base_inflation_radius * velocity_scale_factor;
    
    // Layer 1: Direct cell check with extra conservative threshold for high-speed
    int mx = static_cast<int>((x - map->info.origin.position.x) / map->info.resolution);
    int my = static_cast<int>((y - map->info.origin.position.y) / map->info.resolution);
    
    if (mx < 0 || mx >= static_cast<int>(map->info.width) || 
        my < 0 || my >= static_cast<int>(map->info.height)) {
        return true;
    }
    
    int center_index = my * map->info.width + mx;
    
    // Ultra-conservative threshold for high-speed scenarios
    int occupancy_threshold = robot_velocity > 3.0 ? 5 : 10;
    if (map->data[center_index] > occupancy_threshold) {
        return true;
    }
    
    // Layer 2: Expanded inflation area with velocity compensation
    int inflation_cells = static_cast<int>(adaptive_inflation_radius / map->info.resolution);
    int occupied_count = 0;
    int total_checked = 0;
    
    // Check larger area for high-speed motion
    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
            if (dx == 0 && dy == 0) continue;
            
            double distance = sqrt(dx*dx + dy*dy) * map->info.resolution;
            if (distance > adaptive_inflation_radius) continue;
            
            int check_mx = mx + dx;
            int check_my = my + dy;
            
            if (check_mx < 0 || check_mx >= static_cast<int>(map->info.width) || 
                check_my < 0 || check_my >= static_cast<int>(map->info.height)) {
                occupied_count++;
                total_checked++;
                continue;
            }
            
            int check_index = check_my * map->info.width + check_mx;
            total_checked++;
            
            // More conservative threshold for high-speed
            int area_threshold = robot_velocity > 3.0 ? 8 : 15;
            if (map->data[check_index] > area_threshold) {
                occupied_count++;
            }
        }
    }
    
    // Layer 3: Velocity-adaptive density check
    if (total_checked > 0) {
        double density_threshold = robot_velocity > 3.0 ? 0.1 : 0.2; // Stricter for high-speed
        double obstacle_density = static_cast<double>(occupied_count) / total_checked;
        if (obstacle_density > density_threshold) {
            return true;
        }
    }
    
    // Layer 4: Extended area check with velocity compensation
    int extended_cells = static_cast<int>((adaptive_inflation_radius * 1.5) / map->info.resolution);
    int extended_occupied = 0;
    int extended_total = 0;
    
    for (int dx = -extended_cells; dx <= extended_cells; dx += 3) { // Larger steps for efficiency
        for (int dy = -extended_cells; dy <= extended_cells; dy += 3) {
            int ext_mx = mx + dx;
            int ext_my = my + dy;
            
            if (ext_mx < 0 || ext_mx >= static_cast<int>(map->info.width) || 
                ext_my < 0 || ext_my >= static_cast<int>(map->info.height)) {
                continue;
            }
            
            int ext_index = ext_my * map->info.width + ext_mx;
            extended_total++;
            
            if (map->data[ext_index] > 25) {
                extended_occupied++;
            }
        }
    }
    
    if (extended_total > 0) {
        double extended_threshold = robot_velocity > 3.0 ? 0.08 : 0.15;
        double extended_density = static_cast<double>(extended_occupied) / extended_total;
        if (extended_density > extended_threshold) {
            return true;
        }
    }
    
    return false;
}

std::vector<pcl::PointXY> filter_map_obstacles(
    const std::vector<pcl::PointXY>& points,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    bool use_velocity_compensation,
    double base_inflation_radius,
    double robot_velocity)
{
    std::vector<pcl::PointXY> filtered_points;
    
    for (const auto& point : points) {
        if (is_point_in_map(point.x, point.y, map)) {
            // Use velocity compensation if enabled, otherwise use standard filtering
            bool is_static_obstacle = use_velocity_compensation ? 
                is_map_obstacle_with_velocity_compensation(point.x, point.y, map, base_inflation_radius, robot_velocity) : 
                is_map_obstacle(point.x, point.y, map, base_inflation_radius);
                
            if (!is_static_obstacle) {
                filtered_points.push_back(point);
            }
        }
    }
    
    return filtered_points;
}

// =============== Helper Functions ===============

double shortest_angular_distance(double from, double to)
{
    double diff = to - from;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
}

double extract_yaw_from_pose(const nav_msgs::msg::Odometry::SharedPtr& pose)
{
    if (!pose) return 0.0;
    
    tf2::Quaternion q;
    tf2::fromMsg(pose->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

// =============== DBSCAN Clustering Algorithm ===============

std::vector<std::vector<pcl::PointXY>> dbscan_clustering(
    const std::vector<pcl::PointXY>& points,
    double eps,
    int min_samples)
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
        std::vector<size_t> neighbors = get_neighbors(points, i, eps);
        
        if (static_cast<int>(neighbors.size()) < min_samples) {
            cluster_id[i] = NOISE;
        } else {
            // Expand cluster
            expand_cluster(points, cluster_id, i, neighbors, current_cluster, eps, min_samples);
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

std::vector<size_t> get_neighbors(const std::vector<pcl::PointXY>& points,
                                  size_t point_idx,
                                  double eps)
{
    std::vector<size_t> neighbors;
    const auto& center = points[point_idx];
    
    for (size_t i = 0; i < points.size(); ++i) {
        if (i == point_idx) continue;
        
        // Use squared distance (faster)
        double dist_sq = pow(center.x - points[i].x, 2) + pow(center.y - points[i].y, 2);
        if (dist_sq <= eps * eps) {
            neighbors.push_back(i);
        }
    }
    
    return neighbors;
}

void expand_cluster(const std::vector<pcl::PointXY>& points,
                    std::vector<int>& cluster_id,
                    size_t point_idx,
                    std::vector<size_t>& seed_set,
                    int cluster_num,
                    double eps,
                    int min_samples)
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
        
        // Find neighbors of current point
        std::vector<size_t> neighbors = get_neighbors(points, current_point, eps);
        
        // If current point has enough neighbors, add them to seed set
        if (static_cast<int>(neighbors.size()) >= min_samples) {
            for (auto neighbor : neighbors) {
                if (cluster_id[neighbor] == UNCLASSIFIED) {
                    seed_set.push_back(neighbor);
                }
            }
        }
    }
}

// =============== Temporal Filtering Utilities ===============

std::vector<pcl::PointXY> apply_temporal_filtering(
    const std::vector<pcl::PointXY>& current_points,
    std::vector<std::vector<pcl::PointXY>>& previous_obstacle_history,
    double consistency_threshold,
    int min_frame_presence,
    int max_history_frames)
{
    std::vector<pcl::PointXY> filtered_points;
    
    // Initialize on first run
    if (previous_obstacle_history.empty()) {
        previous_obstacle_history.push_back(current_points);
        // Return all points on first frame (no history to compare against)
        return current_points;
    }
    
    // Temporal consistency filtering
    for (const auto& current_point : current_points) {
        int presence_count = 1; // Current frame counts as 1
        
        // Check presence in previous frames
        for (const auto& prev_frame : previous_obstacle_history) {
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
        if (presence_count >= min_frame_presence && 
            presence_count < static_cast<int>(previous_obstacle_history.size() + 1)) {
            filtered_points.push_back(current_point);
        } else if (previous_obstacle_history.size() < 3) {
            // In early frames, be more permissive
            filtered_points.push_back(current_point);
        }
        // Points that appear in ALL frames are likely static misclassifications, so we filter them out
    }
    
    // Update history (keep only last max_history_frames frames)
    previous_obstacle_history.push_back(current_points);
    if (static_cast<int>(previous_obstacle_history.size()) > max_history_frames) {
        previous_obstacle_history.erase(previous_obstacle_history.begin());
    }
    
    return filtered_points;
}

// =============== High-Speed Motion Filtering ===============

double update_robot_velocity(const nav_msgs::msg::Odometry::SharedPtr& current_pose,
                             std::vector<geometry_msgs::msg::Point>& previous_positions,
                             double update_rate,
                             size_t max_history_positions)
{
    if (!current_pose) {
        return 0.0;
    }
    
    // Update position history
    geometry_msgs::msg::Point current_pos = current_pose->pose.pose.position;
    previous_positions.push_back(current_pos);
    
    // Keep only last max_history_positions positions for velocity calculation
    if (previous_positions.size() > max_history_positions) {
        previous_positions.erase(previous_positions.begin());
    }
    
    // Calculate velocity from position history
    if (previous_positions.size() >= 2) {
        auto& prev_pos = previous_positions[previous_positions.size() - 2];
        auto& curr_pos = previous_positions.back();
        
        double distance = sqrt(pow(curr_pos.x - prev_pos.x, 2) + pow(curr_pos.y - prev_pos.y, 2));
        double time_delta = 1.0 / update_rate; // Approximate time delta
        double velocity = distance / time_delta;
        
        // Smooth velocity using moving average
        static std::vector<double> velocity_history;
        velocity_history.push_back(velocity);
        if (velocity_history.size() > 3) {
            velocity_history.erase(velocity_history.begin());
        }
        
        double sum = 0.0;
        for (double v : velocity_history) {
            sum += v;
        }
        return sum / velocity_history.size();
    }
    
    return 0.0;
}

std::vector<pcl::PointXY> apply_velocity_adaptive_filtering(
    const std::vector<pcl::PointXY>& points,
    const nav_msgs::msg::Odometry::SharedPtr& robot_pose,
    double robot_velocity,
    const std::vector<std::vector<pcl::PointXY>>& previous_obstacle_history,
    int max_persistence_frames,
    double min_movement_threshold)
{
    std::vector<pcl::PointXY> filtered_points;
    
    if (!robot_pose) {
        return points; // No pose data, return as-is
    }
    
    double robot_x = robot_pose->pose.pose.position.x;
    double robot_y = robot_pose->pose.pose.position.y;
    
    // High-speed specific filtering
    for (const auto& point : points) {
        bool should_keep = true;
        
        // Filter 1: Distance-based filtering with velocity scaling
        double distance_to_robot = sqrt(pow(point.x - robot_x, 2) + pow(point.y - robot_y, 2));
        double min_distance_threshold = 0.5 + (robot_velocity * 0.2); // Increase with speed
        
        if (distance_to_robot < min_distance_threshold) {
            should_keep = false; // Too close, likely sensor noise or ground reflection
        }
        
        // Filter 2: Velocity-based direction filtering
        if (robot_velocity > 2.0) { // Only for high-speed motion
            // Calculate direction from robot to obstacle
            double dx = point.x - robot_x;
            double dy = point.y - robot_y;
            
            // Check if obstacle is in unexpected positions for high-speed motion
            // (e.g., directly behind the robot - likely false positive from map misalignment)
            double angle_to_obstacle = atan2(dy, dx);
            
            // Get robot heading from pose
            double robot_yaw = extract_yaw_from_pose(robot_pose);
            
            double relative_angle = shortest_angular_distance(robot_yaw, angle_to_obstacle);
            
            // Filter out obstacles directly behind robot during high-speed motion
            if (std::abs(relative_angle) > 2.5 && distance_to_robot < 3.0) { // Behind robot and close
                should_keep = false;
            }
        }
        
        // Filter 3: Persistence check for stationary "obstacles"
        // Check if this obstacle has been stationary relative to the map for too many frames
        if (should_keep && static_cast<int>(previous_obstacle_history.size()) >= max_persistence_frames) {
            int persistence_count = 0;
            double persistence_threshold = min_movement_threshold;
            
            for (const auto& prev_frame : previous_obstacle_history) {
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
            if (persistence_count >= max_persistence_frames - 1) {
                should_keep = false;
            }
        }
        
        if (should_keep) {
            filtered_points.push_back(point);
        }
    }
    
    return filtered_points;
}

// =============== Message Creation Utilities ===============

obstacle_detection_pkg::msg::ObstacleArray create_obstacle_message(
    const std::vector<std::vector<pcl::PointXY>>& clusters,
    const nav_msgs::msg::Odometry::SharedPtr& robot_pose,
    const std::string& frame_id,
    const rclcpp::Time& timestamp)
{
    obstacle_detection_pkg::msg::ObstacleArray msg;
    msg.header.stamp = timestamp;
    msg.header.frame_id = frame_id;
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
        if (robot_pose) {
            double robot_x = robot_pose->pose.pose.position.x;
            double robot_y = robot_pose->pose.pose.position.y;
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

nav_msgs::msg::OccupancyGrid update_occupancy_grid(
    const std::vector<std::vector<pcl::PointXY>>& clusters,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& static_map,
    const rclcpp::Time& timestamp)
{
    nav_msgs::msg::OccupancyGrid updated_map = *static_map;
    updated_map.header.stamp = timestamp;
    
    // Mark obstacle clusters in the grid
    for (const auto& cluster : clusters) {
        for (const auto& point : cluster) {
            int mx = static_cast<int>((point.x - static_map->info.origin.position.x) / static_map->info.resolution);
            int my = static_cast<int>((point.y - static_map->info.origin.position.y) / static_map->info.resolution);
            
            if (mx >= 0 && mx < static_cast<int>(static_map->info.width) && 
                my >= 0 && my < static_cast<int>(static_map->info.height)) {
                
                int index = my * static_map->info.width + mx;
                updated_map.data[index] = 100; // Mark as occupied
            }
        }
    }
    
    return updated_map;
}

visualization_msgs::msg::MarkerArray create_visualization_markers(
    const std::vector<std::vector<pcl::PointXY>>& clusters,
    const std::string& frame_id,
    const rclcpp::Time& timestamp)
{
    visualization_msgs::msg::MarkerArray markers;
    
    // Clear previous markers
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = timestamp;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);
    
    // Create markers for each cluster
    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& cluster = clusters[i];
        
        // Calculate cluster centroid and size
        double sum_x = 0.0, sum_y = 0.0;
        for (const auto& point : cluster) {
            sum_x += point.x;
            sum_y += point.y;
        }
        double centroid_x = sum_x / cluster.size();
        double centroid_y = sum_y / cluster.size();
        
        // Calculate cluster size
        double max_dist = 0.0;
        for (const auto& point : cluster) {
            double dist = sqrt(pow(point.x - centroid_x, 2) + pow(point.y - centroid_y, 2));
            max_dist = std::max(max_dist, dist);
        }
        
        // Create sphere marker for obstacle
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = timestamp;
        marker.ns = "obstacles";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = centroid_x;
        marker.pose.position.y = centroid_y;
        marker.pose.position.z = 0.5; // Elevated for visibility
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = std::max(0.2, max_dist * 2.0); // Minimum size for visibility
        marker.scale.y = std::max(0.2, max_dist * 2.0);
        marker.scale.z = 1.0;
        
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        
        marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        
        markers.markers.push_back(marker);
    }
    
    return markers;
}

} // namespace obstacle_detection_utils