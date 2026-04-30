#pragma once

#include <algorithm>
#include <deque>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "peak_ros/srv/stream_data.hpp"

namespace peak {

class ReconstructionNodelet : public rclcpp::Node {
public:
  explicit ReconstructionNodelet(const rclcpp::NodeOptions &options);

private:
  template <typename ParamType>
  ParamType paramHandler(std::string param_name, ParamType &param_value);
  void initialisePointcloud();
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  bool publishSrvCb(const peak_ros::srv::StreamData::Request::SharedPtr request,
                    peak_ros::srv::StreamData::Response::SharedPtr response);
  void timerCb();

  int32_t rate_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;

  rclcpp::Service<peak_ros::srv::StreamData>::SharedPtr publish_service_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::PointCloud2 point_cloud_;
  std::deque<sensor_msgs::msg::PointCloud2> buffer_;

  bool use_tf_;
  uint32_t b_scan_count_;
  int direction_;
  std::string recon_frame_id_;
  bool live_publish_;
  double recon_const_vel_;
  bool flip_direction_;
  rclcpp::Time prev_observation_time_;

  // TF
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  geometry_msgs::msg::TransformStamped trans_;
};

} // namespace peak