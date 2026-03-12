#include "rclcpp/rclcpp.hpp"
#include "peak_ros/peak_nodelet.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<peak_namespace::PeakNodelet>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
