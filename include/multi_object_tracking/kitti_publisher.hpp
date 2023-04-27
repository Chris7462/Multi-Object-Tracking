#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class KittiPublisher: public rclcpp::Node
{
  public:
    KittiPublisher();
    ~KittiPublisher() = default;
  private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
