// local header
#include "multi_object_tracking/multi_object_tracking.hpp"


MultiObjectTracking::MultiObjectTracking()
  : Node("multi_object_tracking")
{
  detect_subscription_ = create_subscription<tracking_msgs::msg::DetectedObjectList>(
    "detected_object_list", 10, std::bind(&MultiObjectTracking::tracking_callback, this, std::placeholders::_1));
};


void MultiObjectTracking::tracking_callback(const tracking_msgs::msg::DetectedObjectList& det_msg)
{
  RCLCPP_INFO(get_logger(), "In the callback!");
}
