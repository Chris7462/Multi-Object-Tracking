#pragma once

// ros header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// local ros msg
#include "tracking_msgs/msg/detected_object_list.hpp"

// local header
#include "multi_object_tracking/read_param.hpp"


class MultiObjectTracking: public rclcpp::Node
{
  public:
    MultiObjectTracking();
    ~MultiObjectTracking() = default;

    Param param;
    Trac
  private:
    rclcpp::Subscription<tracking_msgs::msg::DetectedObjectList>::SharedPtr detect_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr textmarker_publisher_;

    void tracking_callback(const tracking_msgs::msg::DetectedObjectList& det_msg);
};