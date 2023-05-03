// C++ header
#include <fstream>
#include <sstream>

// opencv header
#include <opencv2/highgui.hpp>

// pcl header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ros header
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>

// local header
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

  // load OXTS data
  declare_parameter("oxts_folder", "");
  std::string oxts_folder {get_parameter("oxts_folder").as_string()};
  const std::filesystem::path oxts_file {data_path/oxts_folder/(sequence+".txt")};
  load_oxts_data(oxts_file);

  // load label data
  declare_parameter("label_folder", "");
  std::string label_folder {get_parameter("label_folder").as_string()};
  const std::filesystem::path label_file {data_path/label_folder/(sequence+".txt")};
  load_label_data(label_file);

  img_publisher_ = create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
  pc_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
  gps_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
  imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
  detect_publisher_ = create_publisher<tracking_msgs::msg::DetectedObjectList>("detected_object_list", 10);
}

void KittiPublisher::run()
{
  timestamp_ = this->get_clock()->now();
  publish_camera_data();
  publish_lidar_data();
  publish_gps_data();
  publish_imu_data();
  publish_detected_object_list();

  ++frame_;
}

void KittiPublisher::find_camera_data_file(const std::filesystem::path& camera_path)
{
  camera_files_.clear();
  if (std::filesystem::exists(camera_path)) {
    for (const auto& entry: std::filesystem::directory_iterator(camera_path)) {
      if (entry.is_regular_file()) {
        camera_files_.push_back(entry.path());
      }
    }
    if (camera_files_.size() != max_frame_) {
      RCLCPP_ERROR(get_logger(), "Camera file size (%ld) does not match the max frame size (%ld).",
        camera_files_.size(), max_frame_);
      rclcpp::shutdown();
    } else {
      std::sort(camera_files_.begin(), camera_files_.end(),
        [](const auto& lhs, const auto& rhs){
          return lhs.string() < rhs.string();
        });
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Fild camera data files failed. Please check your path setting");
    rclcpp::shutdown();
  }
}

void KittiPublisher::find_lidar_data_file(const std::filesystem::path& lidar_path)
{
  lidar_files_.clear();
  if (std::filesystem::exists(lidar_path)) {
    for (const auto& entry: std::filesystem::directory_iterator(lidar_path)) {
      if (entry.is_regular_file()) {
        lidar_files_.push_back(entry.path());
      }
    }
    if (lidar_files_.size() != max_frame_) {
      RCLCPP_ERROR(get_logger(), "LiDAR file size (%ld) does not match the max frame size (%ld).",
        lidar_files_.size(), max_frame_);
      rclcpp::shutdown();
    } else {
      std::sort(lidar_files_.begin(), lidar_files_.end(),
        [](const auto& lhs, const auto& rhs){
          return lhs.string() < rhs.string();
        });
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Fild LiDAR data files failed. Please check your path setting");
    rclcpp::shutdown();
  }
}

void KittiPublisher::load_oxts_data(const std::filesystem::path& oxts_file)
{
  oxts_data_.clear();
  std::ifstream inf {oxts_file};
  if (!inf) {
    RCLCPP_ERROR(get_logger(), "Load GPS/IMU data failed. Please check your path setting");
  } else {
    std::string line;
    while (std::getline(inf, line)) {
      OXTS oxts;
      std::istringstream iss(line);
      iss >> oxts.lat >> oxts.lon >> oxts.alt >> oxts.roll >> oxts.pitch >> oxts.yaw >> oxts.vn
          >> oxts.ve >> oxts.vf >> oxts.vl >> oxts.vu >> oxts.ax >> oxts.ay >> oxts.az >> oxts.af
          >> oxts.al >> oxts.au >> oxts.wx >> oxts.wy >> oxts.wz >> oxts.wf >> oxts.wl >> oxts.wu
          >> oxts.posacc >> oxts.velacc >> oxts.navstat >> oxts.numsats >> oxts.posmode
          >> oxts.velmode >> oxts.orimode;
      oxts_data_.push_back(oxts);
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

void KittiPublisher::publish_camera_data()
{
  cv::Mat rgb_image = cv::imread(camera_files_.at(frame_));
  if (rgb_image.empty()) {
    RCLCPP_ERROR(get_logger(), "Image does not exist. Check your file path!");
    rclcpp::shutdown();
  } else {
    std_msgs::msg::Header hdr;
    hdr.frame_id = "camera_link";
    hdr.stamp = timestamp_;
    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(hdr, "bgr8", rgb_image).toImageMsg();
    img_publisher_->publish(*img_msg);
  }
}

void KittiPublisher::publish_lidar_data()
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  std::ifstream inf(lidar_files_.at(frame_), std::ios::binary);
  if (!inf) {
    RCLCPP_ERROR(get_logger(), "Load LiDAR data failed. Please check your path setting");
  } else {
    inf.seekg(0, std::ios::beg);
    while (inf.good() && !inf.eof()) {
      pcl::PointXYZI point;
      inf.read(reinterpret_cast<char*>(&point.x), 3*sizeof(float));
      inf.read(reinterpret_cast<char*>(&point.intensity), sizeof(float));
      cloud.points.push_back(point);
    }
  }
  sensor_msgs::msg::PointCloud2 pc_msg;
  pcl::toROSMsg(cloud, pc_msg);
  pc_msg.header.frame_id = "lidar_link";
  pc_msg.header.stamp = timestamp_;
  pc_publisher_->publish(pc_msg);
}

void KittiPublisher::publish_gps_data()
{
  sensor_msgs::msg::NavSatFix gps_msg;
  gps_msg.header.frame_id = "gps_link";
  gps_msg.header.stamp = timestamp_;

  gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;

  const auto& oxts_at_frame = oxts_data_.at(frame_);

  gps_msg.latitude = oxts_at_frame.lat;
  gps_msg.longitude = oxts_at_frame.lon;
  gps_msg.altitude = oxts_at_frame.alt;

  gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
  gps_msg.position_covariance[0] = oxts_at_frame.posacc;
  gps_msg.position_covariance[1] = 0.0;
  gps_msg.position_covariance[2] = 0.0;
  gps_msg.position_covariance[3] = 0.0;
  gps_msg.position_covariance[4] = oxts_at_frame.posacc;
  gps_msg.position_covariance[5] = 0.0;
  gps_msg.position_covariance[6] = 0.0;
  gps_msg.position_covariance[7] = 0.0;
  gps_msg.position_covariance[8] = oxts_at_frame.posacc;
  gps_publisher_->publish(gps_msg);
}

void KittiPublisher::publish_imu_data()
{
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.frame_id = "imu_link";
  imu_msg.header.stamp = timestamp_;

  const auto& oxts_at_frame = oxts_data_.at(frame_);
  tf2::Quaternion q;
  q.setRPY(oxts_at_frame.roll, oxts_at_frame.pitch, oxts_at_frame.yaw);
  imu_msg.orientation.w = q.getW();
  imu_msg.orientation.x = q.getX();
  imu_msg.orientation.y = q.getY();
  imu_msg.orientation.z = q.getZ();

  imu_msg.angular_velocity.x = oxts_at_frame.vf;
  imu_msg.angular_velocity.y = oxts_at_frame.vl;
  imu_msg.angular_velocity.z = oxts_at_frame.vu;

  imu_msg.linear_acceleration.x = oxts_at_frame.ax;
  imu_msg.linear_acceleration.y = oxts_at_frame.ay;
  imu_msg.linear_acceleration.z = oxts_at_frame.az;
  imu_publisher_->publish(imu_msg);
}

void KittiPublisher::publish_detected_object_list()
{
  tracking_msgs::msg::DetectedObjectList obj_list_msg = tracking_msgs::msg::DetectedObjectList();
  obj_list_msg.header.frame_id = "base_link";
  obj_list_msg.header.stamp = timestamp_;

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
  detect_publisher_->publish(obj_list_msg);
}
