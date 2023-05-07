#pragma once

// cpp header
#include <unordered_map>

// OpenCV
#include <opencv2/core.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// ros header
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// local ros msg
#include "tracking_msgs/msg/detected_object_list.hpp"

// local header
#include "multi_object_tracking/read_param.hpp"
#include "multi_object_tracking/tracker.hpp"


class MultiObjectTracking: public rclcpp::Node
{
  public:
    MultiObjectTracking();
    ~MultiObjectTracking() = default;

  private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscriber_;
    rclcpp::Subscription<tracking_msgs::msg::DetectedObjectList>::SharedPtr detect_subscriber_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr textmarker_publisher_;

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg);
    void det_callback(const tracking_msgs::msg::DetectedObjectList::SharedPtr det_msg);

    Param param;
    Tracker tracker;

    Eigen::Matrix3d rotZorigin_;
    Eigen::Isometry3d porigin_;
    double oriheading_;
    double orix_;
    double oriy_;

    Eigen::Matrix3d rotZpre_;
	  Eigen::Isometry3d ppre_;
    double preheading_;
    double prex_;
    double prey_;

    cv::RNG rng_;

    float time_;
    double totaltime_;

    double latitude_;
    double longitude_;
    double heading_;

    cv_bridge::CvImagePtr image_ptr_;
    pcl::PointCloud<pcl::PointXYZI> pc_;

    bool init_;
    rclcpp::Time curr_time_;
    rclcpp::Time prev_time_;

    cv::Point cloud2camera(const Eigen::Vector3d& input);
    Eigen::Vector3d camera2cloud(const Eigen::Vector3d& input);

    void draw3dbox(Detect &det, cv::Mat& image, std::vector<int>& color);

    int64_t gtm();
    std::unordered_map<int, std::vector<int>> idcolor;

    size_t max_marker_size_;
};
