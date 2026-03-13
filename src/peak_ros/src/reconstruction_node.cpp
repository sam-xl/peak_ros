#include <rclcpp/rclcpp.hpp>
#include "peak_ros/reconstruction_nodelet.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<reconstruction_namespace::ReconstructionNodelet>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
