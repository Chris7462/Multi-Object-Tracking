#include <string>

#include "multi_object_tracking/kitti_publisher.hpp"


KittiPublisher::KittiPublisher()
  : Node("kitti_publisher_node")
{
  declare_parameter("data_path", "");
  std::filesystem::path data_path {get_parameter("data_path").get_parameter_value().get<std::string>()};

  declare_parameter("sequence", "");
  std::string sequence {get_parameter("sequence").get_parameter_value().get<std::string>()};

  declare_parameter("camera_folder", "");
  std::string camera_folder {get_parameter("camera_folder").get_parameter_value().get<std::string>()};

  declare_parameter("lidar_folder", "");
  std::string lidar_folder {get_parameter("lidar_folder").get_parameter_value().get<std::string>()};

//// load Camera filenames
//const std::filesystem::path camera_path {data_path/"image_02"/sequence};
//std::vector<std::filesystem::path> camera_filenames;
//find_sensor_data_file(camera_path, camera_filenames);

  publisher_ = create_publisher<std_msgs::msg::String>("kitti_data", 10);
  timer_ = create_wall_timer(std::chrono::milliseconds(100),
    std::bind(&KittiPublisher::timer_callback, this));
}

void KittiPublisher::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

void KittiPublisher::find_sensor_data_file(const std::filesystem::path& input_path, std::vector<std::filesystem::path>& filenames)
{
  if (std::filesystem::exists(input_path)) {
    for (const auto& entry: std::filesystem::directory_iterator(input_path)) {
      if (entry.is_regular_file()) {
        filenames.push_back(entry.path().filename());
      }
    }
    if (filenames.size() != maxFrame_) {
      std::cout << "File size (" << filenames.size() << ") does not match the max frame size (" << maxFrame_ << ")." << std::endl;
    } else {
      std::sort(filenames.begin(), filenames.end(),
        [](const auto& lhs, const auto& rhs){
          return lhs.string() < rhs.string();
        });
    }
  } else {
    std::cout << "Find sensor data files failed. Please check your path setting" << std::endl;
    std::abort();
  }
}
