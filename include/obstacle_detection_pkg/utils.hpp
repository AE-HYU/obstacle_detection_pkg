/**
 * @file utils.hpp
 * @brief Utility functions for obstacle detection including DBSCAN clustering and filtering algorithms
 * 
 * This header provides utility functions for:
 * - DBSCAN clustering algorithm implementation
 * - Map-based obstacle filtering with velocity compensation  
 * - Temporal consistency filtering for static object removal
 * - High-speed motion adaptive filtering
 * - LiDAR data processing and coordinate transformations
 * 
 * @author Jisang Yun
 * @license MIT
 */

#ifndef OBSTACLE_DETECTION_PKG__UTILS_HPP_
#define OBSTACLE_DETECTION_PKG__UTILS_HPP_

#include <vector>
#include <pcl/point_types.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <obstacle_detection_pkg/msg/obstacle_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>

namespace obstacle_detection_utils
{

// =============== Data Processing Utilities ===============

/**
 * @brief Convert polar LiDAR scan data to Cartesian 2D points
 * @param scan LiDAR scan message
 * @param min_range Minimum valid range threshold
 * @param max_range Maximum valid range threshold
 * @return Vector of 2D points in laser frame
 */
std::vector<pcl::PointXY> laser_scan_to_points(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan,
    double min_range, 
    double max_range);

/**
 * @brief Transform points from laser frame to global map frame
 * @param points Points in laser frame
 * @param robot_pose Robot pose in global frame
 * @return Points transformed to global frame
 */
std::vector<pcl::PointXY> transform_points_to_global(
    const std::vector<pcl::PointXY>& points,
    const nav_msgs::msg::Odometry::SharedPtr& robot_pose);

// =============== Map-based Filtering Utilities ===============

/**
 * @brief Check if a point lies within the static map boundaries
 * @param x World X coordinate
 * @param y World Y coordinate
 * @param map Static occupancy grid map
 * @return True if point is within map bounds
 */
bool is_point_in_map(double x, double y, const nav_msgs::msg::OccupancyGrid::SharedPtr& map);

/**
 * @brief Standard map obstacle check with multi-layer filtering
 * @param x World X coordinate  
 * @param y World Y coordinate
 * @param map Static occupancy grid map
 * @param inflation_radius Safety margin around obstacles
 * @return True if point is near static obstacle
 */
bool is_map_obstacle(double x, double y, 
                     const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
                     double inflation_radius);

/**
 * @brief Enhanced map obstacle check with velocity compensation for high-speed motion
 * @param x World X coordinate
 * @param y World Y coordinate  
 * @param map Static occupancy grid map
 * @param base_inflation_radius Base safety margin
 * @param robot_velocity Current robot velocity for adaptive margins
 * @return True if point is static obstacle (with velocity-based margins)
 */
bool is_map_obstacle_with_velocity_compensation(double x, double y,
                                                const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
                                                double base_inflation_radius,
                                                double robot_velocity);

/**
 * @brief Filter out points corresponding to static map obstacles
 * @param points Input points to filter
 * @param map Static occupancy grid map
 * @param use_velocity_compensation Enable velocity-based filtering
 * @param base_inflation_radius Base safety margin
 * @param robot_velocity Current robot velocity
 * @return Filtered points (non-static obstacles only)
 */
std::vector<pcl::PointXY> filter_map_obstacles(
    const std::vector<pcl::PointXY>& points,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    bool use_velocity_compensation,
    double base_inflation_radius,
    double robot_velocity);

// =============== DBSCAN Clustering Algorithm ===============

/**
 * @brief Cluster obstacle points using DBSCAN algorithm
 * @param points Vector of obstacle points to cluster
 * @param eps Maximum distance between points in same cluster
 * @param min_samples Minimum points required to form a cluster
 * @return Vector of clusters, each containing grouped obstacle points
 */
std::vector<std::vector<pcl::PointXY>> dbscan_clustering(
    const std::vector<pcl::PointXY>& points,
    double eps,
    int min_samples);

/**
 * @brief Find neighboring points within epsilon distance for DBSCAN
 * @param points All points being clustered
 * @param point_idx Index of center point
 * @param eps Maximum neighbor distance
 * @return Indices of neighboring points
 */
std::vector<size_t> get_neighbors(const std::vector<pcl::PointXY>& points,
                                  size_t point_idx,
                                  double eps);

/**
 * @brief Expand cluster by adding density-reachable points (DBSCAN core algorithm)
 * @param points All points being clustered
 * @param cluster_id Cluster assignment array for all points
 * @param point_idx Index of current point being processed
 * @param seed_set Initial neighborhood of current point
 * @param cluster_num Current cluster number being built
 * @param eps Maximum distance for density reachability
 * @param min_samples Minimum samples for core point determination
 */
void expand_cluster(const std::vector<pcl::PointXY>& points,
                    std::vector<int>& cluster_id,
                    size_t point_idx,
                    std::vector<size_t>& seed_set,
                    int cluster_num,
                    double eps,
                    int min_samples);

// =============== Temporal Filtering Utilities ===============

/**
 * @brief Apply temporal consistency filtering to reduce false positives from static objects
 * @param current_points Current frame obstacle points
 * @param previous_obstacle_history Previous frames obstacle points history
 * @param consistency_threshold Distance threshold for same obstacle across frames
 * @param min_frame_presence Minimum frames for obstacle validation
 * @param max_history_frames Maximum history frames to maintain
 * @return Filtered points showing temporal consistency (likely dynamic)
 */
std::vector<pcl::PointXY> apply_temporal_filtering(
    const std::vector<pcl::PointXY>& current_points,
    std::vector<std::vector<pcl::PointXY>>& previous_obstacle_history,
    double consistency_threshold = 0.8,
    int min_frame_presence = 2,
    int max_history_frames = 5);

// =============== High-Speed Motion Filtering ===============

/**
 * @brief Update robot velocity from position history for adaptive filtering
 * @param current_pose Current robot pose
 * @param previous_positions Previous robot positions history
 * @param update_rate Processing frequency for time delta calculation
 * @param max_history_positions Maximum position history to maintain
 * @return Smoothed robot velocity
 */
double update_robot_velocity(const nav_msgs::msg::Odometry::SharedPtr& current_pose,
                             std::vector<geometry_msgs::msg::Point>& previous_positions,
                             double update_rate,
                             size_t max_history_positions = 5);

/**
 * @brief Apply velocity-adaptive filtering for high-speed motion scenarios
 * @param points Points to filter based on robot velocity and motion
 * @param robot_pose Current robot pose for direction analysis
 * @param robot_velocity Current robot velocity
 * @param previous_obstacle_history Previous obstacle history for persistence check
 * @param max_persistence_frames Maximum frames for persistence check
 * @param min_movement_threshold Minimum movement for dynamic obstacle validation
 * @return Filtered points with velocity-based corrections
 */
std::vector<pcl::PointXY> apply_velocity_adaptive_filtering(
    const std::vector<pcl::PointXY>& points,
    const nav_msgs::msg::Odometry::SharedPtr& robot_pose,
    double robot_velocity,
    const std::vector<std::vector<pcl::PointXY>>& previous_obstacle_history,
    int max_persistence_frames,
    double min_movement_threshold);

// =============== Message Creation Utilities ===============

/**
 * @brief Create obstacle array message from clustered points
 * @param clusters Vector of obstacle clusters
 * @param robot_pose Robot pose for distance calculations
 * @param frame_id Frame ID for message header
 * @param timestamp Timestamp for message header
 * @return ObstacleArray message with detected obstacles
 */
obstacle_detection_pkg::msg::ObstacleArray create_obstacle_message(
    const std::vector<std::vector<pcl::PointXY>>& clusters,
    const nav_msgs::msg::Odometry::SharedPtr& robot_pose,
    const std::string& frame_id,
    const rclcpp::Time& timestamp);

/**
 * @brief Create updated occupancy grid with detected obstacles marked
 * @param clusters Vector of obstacle clusters
 * @param static_map Base static map
 * @param timestamp Timestamp for updated map
 * @return Updated occupancy grid with obstacles marked
 */
nav_msgs::msg::OccupancyGrid update_occupancy_grid(
    const std::vector<std::vector<pcl::PointXY>>& clusters,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& static_map,
    const rclcpp::Time& timestamp);

/**
 * @brief Create visualization markers for RViz display
 * @param clusters Vector of obstacle clusters  
 * @param frame_id Frame ID for markers
 * @param timestamp Timestamp for markers
 * @return MarkerArray for RViz visualization
 */
visualization_msgs::msg::MarkerArray create_visualization_markers(
    const std::vector<std::vector<pcl::PointXY>>& clusters,
    const std::string& frame_id,
    const rclcpp::Time& timestamp);

// =============== Helper Functions ===============

/**
 * @brief Calculate shortest angular distance between two angles
 * @param from Starting angle (radians)
 * @param to Target angle (radians)
 * @return Shortest angular distance (-π to π)
 */
double shortest_angular_distance(double from, double to);

/**
 * @brief Extract yaw angle from quaternion in odometry message
 * @param pose Odometry message containing quaternion
 * @return Yaw angle in radians
 */
double extract_yaw_from_pose(const nav_msgs::msg::Odometry::SharedPtr& pose);

} // namespace obstacle_detection_utils

#endif // OBSTACLE_DETECTION_PKG__UTILS_HPP_