#include "multi_object_tracking/kitti_publisher.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KittiPublisher>();
  rclcpp::Rate rate(10);
  while (rclcpp::ok() && node->frame_ < node->max_frame_) {
    node->run();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
