/**
 * @file obstacle_detector.hpp
 * @brief Real-time obstacle detection node using LiDAR data and DBSCAN clustering
 * 
 * This node performs dynamic obstacle detection by:
 * 1. Processing LiDAR scan data
 * 2. Filtering out static map obstacles 
 * 3. Clustering remaining points using DBSCAN algorithm
 * 4. Publishing detected obstacles and visualization markers
 */

#ifndef OBSTACLE_DETECTOR_HPP_
#define OBSTACLE_DETECTOR_HPP_

// ROS2 core libraries
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Transform libraries
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

// Point Cloud Library (PCL)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

// Custom message types
#include "obstacle_detection_pkg/msg/obstacle_array.hpp"
#include "obstacle_detection_pkg/msg/obstacle.hpp"

/**
 * @class ObstacleDetector
 * @brief Main class for real-time obstacle detection using LiDAR and DBSCAN clustering
 * 
 * This class subscribes to LiDAR scans, robot odometry, and static map data,
 * then performs obstacle detection by filtering static obstacles and clustering
 * dynamic obstacle points using the DBSCAN algorithm.
 */
class ObstacleDetector : public rclcpp::Node
{
public:
    /**
     * @brief Constructor - initializes node, parameters, subscribers, publishers, and timer
     */
    ObstacleDetector();
    
    /**
     * @brief Default destructor
     */
    ~ObstacleDetector() = default;

private:
    // =============== Callback Functions ===============
    
    /**
     * @brief Callback for LiDAR scan messages
     * @param msg LiDAR scan data containing range measurements
     */
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    /**
     * @brief Callback for robot pose/odometry messages
     * @param msg Robot position and orientation data
     */
    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    /**
     * @brief Callback for static map messages
     * @param msg Occupancy grid map for filtering static obstacles
     */
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    // =============== Main Processing Functions ===============
    
    /**
     * @brief Main obstacle detection processing loop (called by timer)
     * Coordinates the entire detection pipeline from scan to publication
     */
    void processObstacleDetection();
    
    /**
     * @brief Convert LiDAR scan data to 2D points in laser frame
     * @param scan LiDAR scan message
     * @return Vector of 2D points in laser coordinate frame
     */
    std::vector<pcl::PointXY> laserScanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    
    /**
     * @brief Transform points from laser frame to global map frame
     * @param points Vector of points in laser frame
     * @return Vector of points transformed to global frame
     */
    std::vector<pcl::PointXY> transformPointsToGlobal(const std::vector<pcl::PointXY>& points);
    
    /**
     * @brief Filter out points that correspond to static map obstacles
     * @param points Vector of points in global frame
     * @return Filtered points representing potential dynamic obstacles
     */
    std::vector<pcl::PointXY> filterMapObstacles(const std::vector<pcl::PointXY>& points);
    
    /**
     * @brief Cluster obstacle points using DBSCAN algorithm
     * @param points Vector of obstacle points to cluster
     * @return Vector of clusters, each containing grouped obstacle points
     */
    std::vector<std::vector<pcl::PointXY>> dbscanClustering(const std::vector<pcl::PointXY>& points);
    
    // =============== DBSCAN Algorithm Helper Methods ===============
    
    /**
     * @brief Find neighboring points within epsilon distance for DBSCAN
     * @param points All points being clustered
     * @param point_idx Index of the point to find neighbors for
     * @return Indices of neighboring points within epsilon distance
     */
    std::vector<size_t> getNeighbors(const std::vector<pcl::PointXY>& points, size_t point_idx);
    
    /**
     * @brief Expand cluster by adding density-reachable points (DBSCAN core algorithm)
     * @param points All points being clustered
     * @param cluster_id Cluster assignment array for all points
     * @param point_idx Starting point index for cluster expansion
     * @param seed_set Points to be processed for cluster expansion
     * @param cluster_num Current cluster number being expanded
     */
    void expandCluster(const std::vector<pcl::PointXY>& points, std::vector<int>& cluster_id, 
                      size_t point_idx, std::vector<size_t>& seed_set, int cluster_num);
    
    // =============== Output Generation Functions ===============
    
    /**
     * @brief Update occupancy grid with detected obstacle clusters
     * @param clusters Vector of detected obstacle clusters
     * @return Updated occupancy grid with obstacles marked
     */
    nav_msgs::msg::OccupancyGrid updateOccupancyGrid(const std::vector<std::vector<pcl::PointXY>>& clusters);
    
    /**
     * @brief Create obstacle array message from detected clusters
     * @param clusters Vector of detected obstacle clusters
     * @return ObstacleArray message containing obstacle information
     */
    obstacle_detection_pkg::msg::ObstacleArray createObstacleMessage(const std::vector<std::vector<pcl::PointXY>>& clusters);
    
    /**
     * @brief Create visualization markers for RViz display
     * @param clusters Vector of detected obstacle clusters
     * @return MarkerArray message for obstacle visualization
     */
    visualization_msgs::msg::MarkerArray createVisualizationMarkers(const std::vector<std::vector<pcl::PointXY>>& clusters);
    
    // =============== Utility Functions ===============
    
    /**
     * @brief Check if a point is within the bounds of the static map
     * @param x X coordinate in global frame
     * @param y Y coordinate in global frame
     * @return True if point is within map boundaries
     */
    bool isPointInMap(double x, double y) const;
    
    /**
     * @brief Check if a point corresponds to a static obstacle in the map
     * @param x X coordinate in global frame
     * @param y Y coordinate in global frame
     * @return True if point is near a static obstacle (within inflation radius)
     */
    bool isMapObstacle(double x, double y) const;
    
    /**
     * @brief Apply temporal consistency filtering to reduce false positives from static objects
     * @param current_points Current frame obstacle points
     * @return Filtered points that show temporal consistency (likely dynamic)
     */
    std::vector<pcl::PointXY> applyTemporalFiltering(const std::vector<pcl::PointXY>& current_points);
    
    /**
     * @brief Apply velocity-adaptive filtering for high-speed motion scenarios
     * @param points Points to filter based on robot velocity and motion
     * @return Filtered points with velocity-based corrections
     */
    std::vector<pcl::PointXY> applyVelocityAdaptiveFiltering(const std::vector<pcl::PointXY>& points);
    
    /**
     * @brief Update robot velocity tracking from odometry data
     */
    void updateRobotVelocity();
    
    /**
     * @brief Enhanced map obstacle check with velocity compensation
     * @param x World X coordinate
     * @param y World Y coordinate  
     * @return True if point is static map obstacle (with velocity-based margins)
     */
    bool isMapObstacleWithVelocityCompensation(double x, double y) const;
    
    // =============== ROS2 Communication Components ===============
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;        ///< LiDAR scan subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;             ///< Robot pose subscriber  
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;         ///< Static map subscriber
    
    // Publishers
    rclcpp::Publisher<obstacle_detection_pkg::msg::ObstacleArray>::SharedPtr obstacles_pub_;        ///< Detected obstacles publisher
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr updated_map_pub_;                   ///< Updated occupancy grid publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;         ///< RViz markers publisher
    
    // Timer for periodic processing
    rclcpp::TimerBase::SharedPtr timer_;                                           ///< Main processing timer
    
    // =============== Transform Handling ===============
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                                   ///< TF2 buffer for coordinate transformations
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                      ///< TF2 listener for transform updates
    
    // =============== Data Storage ===============
    
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;                           ///< Most recent LiDAR scan
    nav_msgs::msg::Odometry::SharedPtr latest_pose_;                               ///< Most recent robot pose
    nav_msgs::msg::OccupancyGrid::SharedPtr static_map_;                           ///< Static environment map
    
    // =============== Algorithm Parameters ===============
    
    // DBSCAN clustering parameters
    double dbscan_eps_;                                                            ///< Maximum distance between points in same cluster (meters)
    int dbscan_min_samples_;                                                       ///< Minimum points required to form a cluster
    
    // Obstacle detection parameters
    double obstacle_threshold_;                                                    ///< Minimum distance from map obstacles (meters)
    double map_inflation_radius_;                                                  ///< Inflation radius around static map obstacles (meters)
    
    // Enhanced filtering parameters
    std::vector<std::vector<pcl::PointXY>> previous_obstacle_points_;             ///< Previous frame obstacle points for temporal consistency
    rclcpp::Time last_processing_time_;                                           ///< Timestamp of last processing for temporal filtering
    
    // High-speed motion filtering parameters
    bool velocity_adaptive_filtering_;                                            ///< Enable velocity-based adaptive filtering
    int max_obstacle_persistence_frames_;                                         ///< Maximum frames obstacle can persist in same location
    double min_obstacle_movement_threshold_;                                      ///< Minimum movement threshold for dynamic obstacles
    
    // Robot state tracking for velocity-based filtering
    double current_robot_velocity_;                                               ///< Current robot linear velocity
    std::vector<geometry_msgs::msg::Point> previous_robot_positions_;             ///< Previous robot positions for velocity calculation
    
    // LiDAR processing parameters
    double max_lidar_range_;                                                       ///< Maximum valid LiDAR range (meters)
    double min_lidar_range_;                                                       ///< Minimum valid LiDAR range (meters)
    
    // Processing parameters
    double update_rate_;                                                           ///< Obstacle detection update frequency (Hz)
    
    // Publishing options
    bool publish_visualization_;                                                   ///< Enable/disable RViz marker publishing
    bool publish_updated_map_;                                                     ///< Enable/disable updated map publishing
    
    // Frame IDs
    std::string global_frame_id_;                                                  ///< Global coordinate frame (typically "map")
    std::string robot_frame_id_;                                                   ///< Robot base coordinate frame (typically "base_link")
    
    // =============== ROS2 Topic Names ===============
    
    std::string scan_topic_;                                                       ///< LiDAR scan input topic name
    std::string odom_topic_;                                                       ///< Robot odometry input topic name  
    std::string map_topic_;                                                        ///< Static map input topic name
    std::string obstacles_topic_;                                                  ///< Obstacle array output topic name
    std::string updated_map_topic_;                                                ///< Updated occupancy grid output topic name
    std::string markers_topic_;                                                    ///< Visualization markers output topic name
    
    // =============== State Management Flags ===============
    
    bool map_received_;                                                            ///< Flag indicating static map has been received
    bool pose_received_;                                                           ///< Flag indicating robot pose has been received
    bool scan_received_;                                                           ///< Flag indicating LiDAR scan has been received
};

#endif // OBSTACLE_DETECTOR_HPP_