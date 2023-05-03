#pragma once

// ros header
#include <rclcpp/rclcpp.hpp>

// local ros msg
#include "tracking_msgs/msg/detected_object_list.hpp"


class MultiObjectTracking: public rclcpp::Node
{
  public:
    MultiObjectTracking();
    ~MultiObjectTracking() = default;

  private:
    void tracking_callback(const tracking_msgs::msg::DetectedObjectList& det_msg);

    rclcpp::Subscription<tracking_msgs::msg::DetectedObjectList>::SharedPtr detect_subscription_;
};