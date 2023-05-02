#include <fstream>
#include <sstream>

#include "multi_object_tracking/kitti_publisher.hpp"


KittiPublisher::KittiPublisher()
  : Node("kitti_publisher_node"), frame_(0)
{
  declare_parameter("data_path", "");
  std::filesystem::path data_path {get_parameter("data_path").as_string()};

  declare_parameter("sequence", "");
  std::string sequence {get_parameter("sequence").as_string()};

  declare_parameter("max_frame", 0);
  max_frame_ = static_cast<size_t>(get_parameter("max_frame").as_int());

  // load Camera filenames
  declare_parameter("camera_folder", "");
  std::string camera_folder {get_parameter("camera_folder").as_string()};
  const std::filesystem::path camera_path {data_path/camera_folder/sequence};
  find_camera_data_file(camera_path);

  // load LiDAR filenames
  declare_parameter("lidar_folder", "");
  std::string lidar_folder {get_parameter("lidar_folder").as_string()};
  const std::filesystem::path lidar_path {data_path/lidar_folder/sequence};
  find_lidar_data_file(lidar_path);

  // load GPS data
  declare_parameter("gps_folder", "");
  std::string gps_folder {get_parameter("gps_folder").as_string()};
  const std::filesystem::path gps_file {data_path/gps_folder/(sequence+".txt")};
  load_gps_data(gps_file);

  // load label data
  declare_parameter("label_folder", "");
  std::string label_folder {get_parameter("label_folder").as_string()};
  const std::filesystem::path label_file {data_path/label_folder/(sequence+".txt")};
  load_label_data(label_file);

  publisher_ = create_publisher<tracking_msgs::msg::DetectedObjectList>("kitti_data", 10);
}

void KittiPublisher::run()
{
  publish_detected_object_list();
  ++frame_;
}

void KittiPublisher::find_camera_data_file(const std::filesystem::path& camera_path)
{
  camera_files_.clear();
  if (std::filesystem::exists(camera_path)) {
    for (const auto& entry: std::filesystem::directory_iterator(camera_path)) {
      if (entry.is_regular_file()) {
        //camera_files_.push_back(entry.path().filename());
        camera_files_.push_back(entry.path());
      }
    }
    if (camera_files_.size() != max_frame_) {
      RCLCPP_ERROR(get_logger(), "Camera file size (%ld) does not match the max frame size (%ld).",
        camera_files_.size(), max_frame_);
      std::abort();
    } else {
      std::sort(camera_files_.begin(), camera_files_.end(),
        [](const auto& lhs, const auto& rhs){
          return lhs.string() < rhs.string();
        });
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Fild camera data files failed. Please check your path setting");
    std::abort();
  }
}

void KittiPublisher::find_lidar_data_file(const std::filesystem::path& lidar_path)
{
  lidar_files_.clear();
  if (std::filesystem::exists(lidar_path)) {
    for (const auto& entry: std::filesystem::directory_iterator(lidar_path)) {
      if (entry.is_regular_file()) {
        //lidar_files_.push_back(entry.path().filename());
        lidar_files_.push_back(entry.path());
      }
    }
    if (lidar_files_.size() != max_frame_) {
      RCLCPP_ERROR(get_logger(), "LiDAR file size (%ld) does not match the max frame size (%ld).",
        lidar_files_.size(), max_frame_);
      std::abort();
    } else {
      std::sort(lidar_files_.begin(), lidar_files_.end(),
        [](const auto& lhs, const auto& rhs){
          return lhs.string() < rhs.string();
        });
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Fild LiDAR data files failed. Please check your path setting");
    std::abort();
  }
}

void KittiPublisher::load_gps_data(const std::filesystem::path& gps_file)
{
  gps_data_.clear();
  std::ifstream inf {gps_file};
  if (!inf) {
    RCLCPP_ERROR(get_logger(), "Load GPS/IMU data failed. Please check your path setting");
  } else {
    std::string line;
    while (std::getline(inf, line)) {
      GPSIMU gi;
      std::istringstream iss(line);
      iss >> gi.lat >> gi.lon >> gi.alt >> gi.roll >> gi.pitch >> gi.yaw >> gi.vn
          >> gi.ve >> gi.vf >> gi.vl >> gi.vu >> gi.ax >> gi.ay >> gi.az >> gi.af
          >> gi.al >> gi.au >> gi.wx >> gi.wy >> gi.wz >> gi.wf >> gi.wl >> gi.wu
          >> gi.posacc >> gi.velacc >> gi.navstat >> gi.numsats >> gi.posmode
          >> gi.velmode >> gi.orimode;
      gps_data_.push_back(gi);
    }
  }
}

void KittiPublisher::load_label_data(const std::filesystem::path& label_file)
{
  label_data_.clear();
  label_data_.resize(max_frame_, std::vector<LABEL>());

  std::ifstream inf {label_file};
  if (!inf) {
    RCLCPP_ERROR(get_logger(), "Load label data failed. Please check your path setting");
  } else {
    std::string line;
    while (std::getline(inf, line)) {
      LABEL obj;
      int frame;
      int track_id;
      std::istringstream iss(line);
      iss >> frame >> track_id >> obj.label >> obj.truncation >> obj.occlusion
          >> obj.alpha >> obj.left >> obj.top >> obj.right >> obj.bottom
          >> obj.height >> obj.width >> obj.length >> obj.x >> obj.y
          >> obj.z >> obj.rotation_y;

      // skip for largely truncated, largely occluded, or not Car type object
      if (obj.truncation > maxTruncation || obj.occlusion > maxOcclusion || obj.label != "Car") {
        continue;
      } else {
        label_data_.at(frame).push_back(obj);
      }
    }
  }
}

void KittiPublisher::publish_detected_object_list()
{
  tracking_msgs::msg::DetectedObjectList obj_list_msg = tracking_msgs::msg::DetectedObjectList();
  obj_list_msg.header.frame_id = "map";
  obj_list_msg.header.stamp = get_clock()->now();

  for (auto& obj: label_data_.at(frame_)) {
    auto obj_msg = tracking_msgs::msg::DetectedObject();
    obj_msg.label = obj.label;
    obj_msg.left = obj.left;
    obj_msg.top = obj.top;
    obj_msg.right = obj.right;
    obj_msg.bottom = obj.bottom;
    obj_msg.height = obj.height;
    obj_msg.width = obj.width;
    obj_msg.length = obj.length;
    obj_msg.x = obj.x;
    obj_msg.y = obj.y;
    obj_msg.z = obj.z;
    obj_msg.yaw = obj.rotation_y;

    obj_list_msg.data.push_back(obj_msg);
  }
  //RCLCPP_INFO(this->get_logger(), "Publishing: frame %ld data", frame_);
  publisher_->publish(obj_list_msg);
}
