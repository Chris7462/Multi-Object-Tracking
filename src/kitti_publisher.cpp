#include "multi_object_tracking/kitti_publisher.hpp"

KittiPublisher::KittiPublisher()
  : Node("kitti_publisher")
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("kitti_data", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
    std::bind(&KittiPublisher::timer_callback, this));
}

void KittiPublisher::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}
