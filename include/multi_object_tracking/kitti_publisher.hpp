#pragma once
#include <filesystem>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// local ros msg
#include "tracking_msgs/msg/detected_object_list.hpp"

// local header
#include "multi_object_tracking/helper.hpp"

class KittiPublisher: public rclcpp::Node
{
  public:
    KittiPublisher();
    ~KittiPublisher() = default;

  private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<tracking_msgs::msg::DetectedObjectList>::SharedPtr publisher_;

    std::vector<std::filesystem::path> camera_filenames_;
    std::vector<std::filesystem::path> lidar_filenames_;
    std::vector<GPSIMU> gps_data_;
    std::vector<std::vector<LABEL>> label_data_;

    size_t max_frame_;
    void find_camera_data_file(const std::filesystem::path& camera_path);
    void find_lidar_data_file(const std::filesystem::path& lidar_path);
    void load_gps_data(const std::filesystem::path& gps_file);
    void load_label_data(const std::filesystem::path& label_file);
};
