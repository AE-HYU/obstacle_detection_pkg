#ifndef OBSTACLE_DETECTOR_HPP_
#define OBSTACLE_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

#include "obstacle_detection_pkg/msg/obstacle_array.hpp"
#include "obstacle_detection_pkg/msg/obstacle.hpp"

class ObstacleDetector : public rclcpp::Node
{
public:
    ObstacleDetector();
    ~ObstacleDetector() = default;

private:
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    void processObstacleDetection();
    std::vector<pcl::PointXY> laserScanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    std::vector<pcl::PointXY> transformPointsToGlobal(const std::vector<pcl::PointXY>& points);
    std::vector<pcl::PointXY> filterMapObstacles(const std::vector<pcl::PointXY>& points);
    std::vector<std::vector<pcl::PointXY>> dbscanClustering(const std::vector<pcl::PointXY>& points);
    
    // DBSCAN helper methods
    std::vector<size_t> getNeighbors(const std::vector<pcl::PointXY>& points, size_t point_idx);
    void expandCluster(const std::vector<pcl::PointXY>& points, std::vector<int>& cluster_id, 
                      size_t point_idx, std::vector<size_t>& seed_set, int cluster_num);
    
    nav_msgs::msg::OccupancyGrid updateOccupancyGrid(const std::vector<std::vector<pcl::PointXY>>& clusters);
    obstacle_detection_pkg::msg::ObstacleArray createObstacleMessage(const std::vector<std::vector<pcl::PointXY>>& clusters);
    visualization_msgs::msg::MarkerArray createVisualizationMarkers(const std::vector<std::vector<pcl::PointXY>>& clusters);
    
    bool isPointInMap(double x, double y) const;
    bool isMapObstacle(double x, double y) const;
    
    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    
    rclcpp::Publisher<obstacle_detection_pkg::msg::ObstacleArray>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr updated_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Transform handling
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Data storage
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    nav_msgs::msg::Odometry::SharedPtr latest_pose_;
    nav_msgs::msg::OccupancyGrid::SharedPtr static_map_;
    
    // Parameters
    double dbscan_eps_;
    int dbscan_min_samples_;
    double obstacle_threshold_;
    double map_inflation_radius_;
    double max_lidar_range_;
    double min_lidar_range_;
    double update_rate_;
    bool publish_visualization_;
    bool publish_updated_map_;
    std::string global_frame_id_;
    std::string robot_frame_id_;
    
    // Topic names
    std::string scan_topic_;
    std::string odom_topic_;
    std::string map_topic_;
    std::string obstacles_topic_;
    std::string updated_map_topic_;
    std::string markers_topic_;
    
    // State flags
    bool map_received_;
    bool pose_received_;
    bool scan_received_;
};

#endif // OBSTACLE_DETECTOR_HPP_