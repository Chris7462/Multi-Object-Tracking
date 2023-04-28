#include "multi_object_tracking/kitti_publisher.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KittiPublisher>());
  rclcpp::shutdown();

  return 0;
}
