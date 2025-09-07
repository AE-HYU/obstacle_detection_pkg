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
#include "obstacle_detection_pkg/utils.hpp"

// Use obstacle_detection_utils namespace for all utility functions
using namespace obstacle_detection_utils;

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
    this->declare_parameter("obstacle_threshold", 0.02);
    this->declare_parameter("map_inflation_radius", 0.8);
    
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
    RCLCPP_INFO(this->get_logger(), "Starting obstacle detector node...");
}

// =============== ROS2 Callback Functions ===============

/**
 * @brief Store latest LiDAR scan data and mark as received
 */
void ObstacleDetector::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    latest_scan_ = msg;
    scan_received_ = true;
}

/**
 * @brief Store latest robot pose/odometry data and mark as received
 */
void ObstacleDetector::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    latest_pose_ = msg;
    pose_received_ = true;
}

/**
 * @brief Store static map data and mark as received
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
 * This function coordinates the complete obstacle detection pipeline using utility functions:
 * 1. Checks that all required data (map, pose, scan) is available
 * 2. Updates robot velocity for adaptive filtering
 * 3. Converts LiDAR scan to 2D points using laser_scan_to_points()
 * 4. Transforms points to global frame using transform_points_to_global()
 * 5. Filters static map obstacles using filter_map_obstacles()  
 * 6. Applies temporal filtering using apply_temporal_filtering()
 * 7. Applies velocity-adaptive filtering using apply_velocity_adaptive_filtering()
 * 8. Clusters remaining points using dbscan_clustering()
 * 9. Creates and publishes results using utility message creation functions
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
        current_robot_velocity_ = update_robot_velocity(latest_pose_, previous_robot_positions_, update_rate_);
        
        // Step 1: Convert LiDAR scan to 2D points in laser coordinate frame
        auto points = laser_scan_to_points(latest_scan_, min_lidar_range_, max_lidar_range_);
        
        // Step 2: Transform points from laser frame to global map frame
        auto global_points = transform_points_to_global(points, latest_pose_);
        
        // Step 3: Filter out points corresponding to static map obstacles with velocity compensation
        auto obstacle_points = filter_map_obstacles(global_points, static_map_, 
                                                    velocity_adaptive_filtering_, 
                                                    map_inflation_radius_, 
                                                    current_robot_velocity_);
        
        // Step 3.5: Apply temporal consistency filtering for additional robustness
        obstacle_points = apply_temporal_filtering(obstacle_points, previous_obstacle_points_);
        
        // Step 3.7: Apply velocity-adaptive filtering for high-speed motion
        if (velocity_adaptive_filtering_) {
            obstacle_points = apply_velocity_adaptive_filtering(obstacle_points, latest_pose_,
                                                                current_robot_velocity_, 
                                                                previous_obstacle_points_,
                                                                max_obstacle_persistence_frames_,
                                                                min_obstacle_movement_threshold_);
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
        auto clusters = dbscan_clustering(obstacle_points, dbscan_eps_, dbscan_min_samples_);
        
        // Step 5: Create and publish obstacle detection results
        auto obstacle_msg = create_obstacle_message(clusters, latest_pose_, global_frame_id_, this->now());
        obstacles_pub_->publish(obstacle_msg);
        
        // Step 6: Publish optional outputs if enabled
        if (publish_updated_map_) {
            auto updated_map = update_occupancy_grid(clusters, static_map_, this->now());
            updated_map_pub_->publish(updated_map);
        }
        
        if (publish_visualization_) {
            auto markers = create_visualization_markers(clusters, global_frame_id_, this->now());
            visualization_pub_->publish(markers);
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Successfully detected %zu obstacle clusters", clusters.size());
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Error in obstacle detection pipeline: %s", e.what());
    }
}

// =============== Main Entry Point ===============

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ObstacleDetector>();
    
    RCLCPP_INFO(node->get_logger(), "Starting obstacle detector node...");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

// =============== End of Implementation ===============
// All utility functions have been moved to utils.cpp for better code organization