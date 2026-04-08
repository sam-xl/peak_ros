#include "peak_ros/reconstruction_nodelet.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<reconstruction_namespace::ReconstructionNodelet>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
