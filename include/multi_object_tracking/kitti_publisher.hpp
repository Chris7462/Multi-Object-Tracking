#pragma once
#include <filesystem>
#include <vector>

// ros header
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// local header
#include "multi_object_tracking/helper.hpp"

// local ros msg
#include "tracking_msgs/msg/detected_object_list.hpp"

class KittiPublisher: public rclcpp::Node
{
  public:
    KittiPublisher();
    ~KittiPublisher() = default;

    void init();
    void run();
    inline size_t frame() {return frame_;}
    inline size_t max_frame() {return max_frame_;}

  private:
    std::shared_ptr<image_transport::ImageTransport> it_;
    std::shared_ptr<image_transport::Publisher> img_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
    rclcpp::Publisher<tracking_msgs::msg::DetectedObjectList>::SharedPtr detect_publisher_;

    std::vector<std::filesystem::path> camera_files_;
    std::vector<std::filesystem::path> lidar_files_;
    std::vector<GPSIMU> gps_data_;
    std::vector<std::vector<LABEL>> label_data_;

    rclcpp::Time timestamp_;
    size_t frame_;
    size_t max_frame_;

    void find_camera_data_file(const std::filesystem::path& camera_path);
    void find_lidar_data_file(const std::filesystem::path& lidar_path);
    void load_gps_data(const std::filesystem::path& gps_file);
    void load_label_data(const std::filesystem::path& label_file);

    void publish_camera_data();
    void publish_lidar_data();
    void publish_detected_object_list();
};
