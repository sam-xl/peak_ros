#pragma once

#include <cmath>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.h>

#include "PeakMicroPulseHandler/peak_handler.h"

#include "peak_ros/msg/ascan.hpp"
#include "peak_ros/msg/observation.hpp"

#include "peak_ros/srv/send_command.hpp"
#include "peak_ros/srv/stream_data.hpp"
#include "peak_ros/srv/take_single_measurement.hpp"

namespace peak {

class PeakNodelet : public rclcpp::Node {
public:
  PeakNodelet(const rclcpp::NodeOptions &options);
  //~PeakNode();

private:
  template <typename ParamType>
  ParamType paramHandler(std::string param_name, ParamType &param_value);

  void initHardware();
  void prePopulateAScanMessage();
  void prePopulateBScanMessage();
  void prePopulateGatedBScanMessage();
  void sendCommand(const std::string &cmd);
  bool
  streamDataSrvCb(const peak_ros::srv::StreamData::Request::SharedPtr request,
                  peak_ros::srv::StreamData::Response::SharedPtr response);
  bool takeMeasurementSrvCb(
      const peak_ros::srv::TakeSingleMeasurement::Request::SharedPtr request,
      peak_ros::srv::TakeSingleMeasurement::Response::SharedPtr response);
  bool
  sendCommandSrvCb(const peak_ros::srv::SendCommand::Request::SharedPtr request,
                   peak_ros::srv::SendCommand::Response::SharedPtr response);
  void takeMeasurement();
  void populateAScanMessage();
  void populateBScanMessage(const peak_ros::msg::Observation &obs_msg);
  void timerCb();

  void
  postSetParametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  int digitisation_rate_;
  std::string package_path_;
  bool profile_;

  // Config
  int acquisition_rate_;
  std::string peak_address_;
  int peak_port_;
  std::string mps_path_;

  // LTPA parameters
  int ltpa_gate_start_ = -1;
  int ltpa_gate_end_ = -1;
  int ltpa_delay_ = -1;
  int ltpa_gain_ = -1;
  int ltpa_prf_ = -1;

  // TCG
  bool use_tcg_;
  float amp_factor_;
  float depth_factor_;
  float tcg_limit_;

  // Gates
  float gate_front_wall_;
  float depth_to_skip_;
  float gate_back_wall_;
  float max_depth_;
  bool zero_to_front_wall_;
  bool show_front_wall_;

  // Input
  PeakHandler peak_handler_;
  const PeakHandler::OutputFormat *ltpa_data_ptr_;

  // Output
  peak_ros::msg::Observation ltpa_msg_;
  sensor_msgs::msg::PointCloud2 bscan_cloud_;
  sensor_msgs::msg::PointCloud2 gated_bscan_cloud_;

  bool stream_;

  rclcpp::Publisher<peak_ros::msg::Observation>::SharedPtr ascan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr bscan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      gated_bscan_publisher_;

  rclcpp::Service<peak_ros::srv::TakeSingleMeasurement>::SharedPtr
      single_measure_service_;
  rclcpp::Service<peak_ros::srv::StreamData>::SharedPtr stream_service_;
  rclcpp::Service<peak_ros::srv::SendCommand>::SharedPtr send_command_service_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Node::PostSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

} // namespace peak