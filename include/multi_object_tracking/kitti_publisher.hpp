#pragma once
#include <filesystem>

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

    size_t maxFrame_;
    void find_sensor_data_file(const std::filesystem::path& input_path, std::vector<std::filesystem::path>& filenames);
};
