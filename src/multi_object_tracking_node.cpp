#include "multi_object_tracking/multi_object_tracking.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiObjectTracking>());
  rclcpp::shutdown();

  return 0;
}
