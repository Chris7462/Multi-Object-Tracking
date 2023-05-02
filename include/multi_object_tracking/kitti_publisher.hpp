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

    void run();

    size_t frame_;
    size_t max_frame_;

  private:
    rclcpp::Publisher<tracking_msgs::msg::DetectedObjectList>::SharedPtr publisher_;

    std::vector<std::filesystem::path> camera_files_;
    std::vector<std::filesystem::path> lidar_files_;
    std::vector<GPSIMU> gps_data_;
    std::vector<std::vector<LABEL>> label_data_;

    void find_camera_data_file(const std::filesystem::path& camera_path);
    void find_lidar_data_file(const std::filesystem::path& lidar_path);
    void load_gps_data(const std::filesystem::path& gps_file);
    void load_label_data(const std::filesystem::path& label_file);

    void publish_detected_object_list();
};
