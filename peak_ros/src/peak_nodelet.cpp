#include "peak_ros/peak_nodelet.h"

namespace peak {

PeakNodelet::PeakNodelet(const rclcpp::NodeOptions &options)
    : rclcpp::Node("peak_node", options), peak_handler_(), stream_(false) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Initialising node...");

  peak_handler_.setup(
      PeakNodelet::paramHandler("acquisition_rate", acquisition_rate_),
      PeakNodelet::paramHandler("peak_address", peak_address_),
      PeakNodelet::paramHandler("peak_port", peak_port_),
      PeakNodelet::paramHandler("mps_path", mps_path_));

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // TODO: Move to using smart pointers, mutex, futures or semiphors
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  ltpa_data_ptr_ = peak_handler_.ltpa_data_ptr();

  PeakNodelet::paramHandler("digitisation_rate", digitisation_rate_);
  PeakNodelet::paramHandler("profile", profile_);

  PeakNodelet::paramHandler("tcg.use_tcg", use_tcg_);
  PeakNodelet::paramHandler("tcg.amp_factor", amp_factor_);
  PeakNodelet::paramHandler("tcg.depth_factor", depth_factor_);
  PeakNodelet::paramHandler("tcg.tcg_limit", tcg_limit_);
  depth_factor_ = depth_factor_ * 0.001f;

  PeakNodelet::paramHandler("gates.gate_front_wall", gate_front_wall_);
  PeakNodelet::paramHandler("gates.depth_to_skip", depth_to_skip_);
  PeakNodelet::paramHandler("gates.gate_back_wall", gate_back_wall_);
  PeakNodelet::paramHandler("gates.max_depth", max_depth_);
  PeakNodelet::paramHandler("gates.zero_to_front_wall", zero_to_front_wall_);
  PeakNodelet::paramHandler("gates.show_front_wall", show_front_wall_);
  depth_to_skip_ = depth_to_skip_ * 0.001f;
  max_depth_ = max_depth_ * 0.001f;

  initHardware();

  prePopulateAScanMessage();
  prePopulateBScanMessage();
  prePopulateGatedBScanMessage();

  ascan_publisher_ =
      this->create_publisher<peak_ros::msg::Observation>("a_scans", 100);
  bscan_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("b_scan", 100);
  gated_bscan_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("gated_b_scan",
                                                            100);

  single_measure_service_ =
      this->create_service<peak_ros::srv::TakeSingleMeasurement>(
          "take_single_measurement",
          std::bind(&PeakNodelet::takeMeasurementSrvCb, this,
                    std::placeholders::_1, std::placeholders::_2));
  stream_service_ = this->create_service<peak_ros::srv::StreamData>(
      "stream_data", std::bind(&PeakNodelet::streamDataSrvCb, this,
                               std::placeholders::_1, std::placeholders::_2));

  send_command_service_ = this->create_service<peak_ros::srv::SendCommand>(
      "send_command", std::bind(&PeakNodelet::sendCommandSrvCb, this,
                                std::placeholders::_1, std::placeholders::_2));

  timer_ = this->create_wall_timer(
      std::chrono::nanoseconds(1000000000 / acquisition_rate_),
      std::bind(&PeakNodelet::timerCb, this));

  RCLCPP_INFO_STREAM(this->get_logger(), "Node initialised");
}

// PeakNodelet::~PeakNodelet() {
// }

template <typename ParamType>
ParamType PeakNodelet::paramHandler(std::string param_name,
                                    ParamType &param_value) {
  if (this->has_parameter(param_name)) {
    param_value = this->get_parameter(param_name).get_value<ParamType>();
  } else {
    param_value = this->declare_parameter(param_name, param_value);
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Read in parameter " << param_name
                                                              << " = "
                                                              << param_value);

  return param_value;
}

void PeakNodelet::initHardware() {
  RCLCPP_INFO_STREAM(this->get_logger(), "Initialising Peak hardware...");

  peak_handler_.connect();
  peak_handler_.sendReset(digitisation_rate_);
  peak_handler_.readMpsFile();
  peak_handler_.sendMpsConfiguration();

  RCLCPP_INFO_STREAM(this->get_logger(), "Peak hardware initialised");
}

void PeakNodelet::prePopulateAScanMessage() {
  PeakNodelet::paramHandler("frame_id", ltpa_msg_.header.frame_id);

  // Get settings the PeakHandler extracted from the .mps file
  ltpa_msg_.dof = peak_handler_.dof_;
  ltpa_msg_.gate_start = peak_handler_.gate_start_;
  ltpa_msg_.gate_end = peak_handler_.gate_end_;
  ltpa_msg_.ascan_length = peak_handler_.ascan_length_;
  ltpa_msg_.num_ascans = peak_handler_.num_a_scans_;
  ltpa_msg_.ascans.reserve(ltpa_msg_.num_ascans);

  ltpa_msg_.digitisation_rate = ltpa_data_ptr_->digitisation_rate;

  // TODO: Consider sending this as a separate one time latched message rather
  // than repeated here
  PeakNodelet::paramHandler("boundary_conditions.n_elements",
                            ltpa_msg_.n_elements);
  PeakNodelet::paramHandler("boundary_conditions.element_pitch",
                            ltpa_msg_.element_pitch);
  PeakNodelet::paramHandler("boundary_conditions.inter_element_spacing",
                            ltpa_msg_.inter_element_spacing);
  PeakNodelet::paramHandler("boundary_conditions.element_width",
                            ltpa_msg_.element_width);
  PeakNodelet::paramHandler("boundary_conditions.vel_wedge",
                            ltpa_msg_.vel_wedge);
  PeakNodelet::paramHandler("boundary_conditions.vel_couplant",
                            ltpa_msg_.vel_couplant);
  PeakNodelet::paramHandler("boundary_conditions.vel_material",
                            ltpa_msg_.vel_material);
  PeakNodelet::paramHandler("boundary_conditions.wedge_angle",
                            ltpa_msg_.wedge_angle);
  PeakNodelet::paramHandler("boundary_conditions.wedge_depth",
                            ltpa_msg_.wedge_depth);
  PeakNodelet::paramHandler("boundary_conditions.couplant_depth",
                            ltpa_msg_.couplant_depth);
  PeakNodelet::paramHandler("boundary_conditions.specimen_depth",
                            ltpa_msg_.specimen_depth);

  // Not entirely necessary as implemented here but we can pass these back to
  // the peak_handler
  peak_handler_.setReconstructionConfiguration(
      ltpa_msg_.n_elements, ltpa_msg_.element_pitch,
      ltpa_msg_.inter_element_spacing, ltpa_msg_.element_width,
      ltpa_msg_.vel_wedge, ltpa_msg_.vel_couplant, ltpa_msg_.vel_material,
      ltpa_msg_.wedge_angle, ltpa_msg_.wedge_depth, ltpa_msg_.couplant_depth,
      ltpa_msg_.specimen_depth);
}

void PeakNodelet::prePopulateBScanMessage() {
  int fields = 4;
  int bytes_per_field = 4; // 32 bits = 4 bytes

  sensor_msgs::PointCloud2Modifier bscan_cloud_modifier(bscan_cloud_);
  bscan_cloud_modifier.setPointCloud2Fields(
      fields, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "Amplitudes", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  bscan_cloud_.height = 1;
  bscan_cloud_.is_dense = true;
  bscan_cloud_.point_step = fields * bytes_per_field;
}

void PeakNodelet::prePopulateGatedBScanMessage() {
  int fields = 5;
  int bytes_per_field = 4; // 32 bits = 4 bytes

  sensor_msgs::PointCloud2Modifier gated_bscan_cloud_modifier(
      gated_bscan_cloud_);
  gated_bscan_cloud_modifier.setPointCloud2Fields(
      fields, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "Amplitudes", 1,
      sensor_msgs::msg::PointField::FLOAT32, "TimeofFlight", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  gated_bscan_cloud_.height = 1;
  gated_bscan_cloud_.is_dense = true;
  gated_bscan_cloud_.point_step = fields * bytes_per_field;
}

bool PeakNodelet::streamDataSrvCb(
    const peak_ros::srv::StreamData::Request::SharedPtr request,
    peak_ros::srv::StreamData::Response::SharedPtr response) {
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Streaming request received: " << request->stream_data);
  if (request->stream_data) {
    stream_ = true;
    response->success = true;
    return true;
  } else {
    stream_ = false;
    response->success = true;
    return true;
  }
}

bool PeakNodelet::takeMeasurementSrvCb(
    const peak_ros::srv::TakeSingleMeasurement::Request::SharedPtr request,
    peak_ros::srv::TakeSingleMeasurement::Response::SharedPtr response) {
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Take single measurement request received: "
                         << request->take_single_measurement);
  if (request->take_single_measurement) {
    takeMeasurement();
    response->success = true;
    return true;
  } else {
    response->success = false;
    return true;
  }
}

bool PeakNodelet::sendCommandSrvCb(
    const peak_ros::srv::SendCommand::Request::SharedPtr request,
    peak_ros::srv::SendCommand::Response::SharedPtr response) {
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Sending command: " << request->command);
  peak_handler_.sendCommand(request->command);

  // Update packet length if necessary
  if (request->command.rfind("GATS", 0) == 0) {
    peak_handler_.setGates(request->command);
    peak_handler_.calcPacketLength();
    prePopulateAScanMessage();
    prePopulateBScanMessage();
    prePopulateGatedBScanMessage();
  }

  response->success = true;
  return true;
}

void PeakNodelet::takeMeasurement() {
  // TODO: Remove profiling when happy with acquisition rates
  std::chrono::high_resolution_clock::time_point begin;
  std::chrono::high_resolution_clock::time_point end;
  std::chrono::high_resolution_clock::time_point end_1;
  std::chrono::high_resolution_clock::time_point end_2;
  std::chrono::high_resolution_clock::time_point end_3;
  std::chrono::high_resolution_clock::time_point end_4;
  std::chrono::high_resolution_clock::time_point end_5;

  if (profile_)
    begin = std::chrono::high_resolution_clock::now();

  // ~40ms
  if (peak_handler_.sendDataRequest()) {

    if (profile_) {
      end_1 = std::chrono::high_resolution_clock::now();
      std::cout << "\033[32m";
      std::cout << "Profiling [peak_handler_.sendDataRequest()] --- "
                << std::chrono::duration_cast<std::chrono::microseconds>(end_1 -
                                                                         begin)
                       .count()
                << " us" << std::endl;
      std::cout << "\033[0m";
    }

    // ~0.3ms
    populateAScanMessage();

    if (profile_) {
      end_2 = std::chrono::high_resolution_clock::now();
      std::cout << "\033[32m";
      std::cout << "Profiling [PeakNodelet::populateAScanMessage()] --- "
                << std::chrono::duration_cast<std::chrono::microseconds>(end_2 -
                                                                         end_1)
                       .count()
                << " us" << std::endl;
      std::cout << "\033[0m";
    }

    // ~0.4ms
    ascan_publisher_->publish(ltpa_msg_);

    if (profile_) {
      end_3 = std::chrono::high_resolution_clock::now();
      std::cout << "\033[32m";
      std::cout << "Profiling [ascan_publisher_.publish(ltpa_msg_)] --- "
                << std::chrono::duration_cast<std::chrono::microseconds>(end_3 -
                                                                         end_2)
                       .count()
                << " us" << std::endl;
      std::cout << "\033[0m";
    }

    // ~12ms
    populateBScanMessage(ltpa_msg_);

    if (profile_) {
      end_4 = std::chrono::high_resolution_clock::now();
      std::cout << "\033[32m";
      std::cout
          << "Profiling [PeakNodelet::populateBScanMessage(ltpa_msg_)] --- "
          << std::chrono::duration_cast<std::chrono::microseconds>(end_4 -
                                                                   end_3)
                 .count()
          << " us" << std::endl;
      std::cout << "\033[0m";
    }

    bscan_publisher_->publish(bscan_cloud_);

    if (profile_) {
      end_5 = std::chrono::high_resolution_clock::now();
      std::cout << "\033[32m";
      std::cout << "Profiling [bscan_publisher_.publish(bscan_cloud_);] --- "
                << std::chrono::duration_cast<std::chrono::microseconds>(end_5 -
                                                                         end_4)
                       .count()
                << " us" << std::endl;
      std::cout << "\033[0m";
    }

    gated_bscan_publisher_->publish(gated_bscan_cloud_);
  }

  if (profile_) {
    end = std::chrono::high_resolution_clock::now();
    std::cout << "\033[32m";
    std::cout << "Profiling [PeakNodelet::takeMeasurement()] --- "
              << std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                       begin)
                     .count()
              << " us" << std::endl;
    std::cout << "\033[0m";
  }
}

void PeakNodelet::populateAScanMessage() {
  ltpa_msg_.header.stamp = this->get_clock()->now();
  ltpa_msg_.ascans.clear();

  for (auto ascan : ltpa_data_ptr_->ascans) {
    peak_ros::msg::Ascan ascan_msg;
    ascan_msg.count = ascan.header.count;
    ascan_msg.test_number = ascan.header.testNo;
    ascan_msg.dof = ascan.header.dof;
    ascan_msg.channel = ascan.header.channel;
    ascan_msg.amplitudes = ascan.amps;
    ltpa_msg_.ascans.push_back(ascan_msg);
  }

  ltpa_msg_.max_amplitude = ltpa_data_ptr_->max_amplitude;
}

void PeakNodelet::populateBScanMessage(
    const peak_ros::msg::Observation &obs_msg) {
  bscan_cloud_.header.stamp = obs_msg.header.stamp;
  bscan_cloud_.header.frame_id = obs_msg.header.frame_id;
  bscan_cloud_.width = obs_msg.ascan_length * obs_msg.num_ascans;
  bscan_cloud_.row_step = bscan_cloud_.point_step * bscan_cloud_.width;
  bscan_cloud_.data.clear();
  bscan_cloud_.data.resize(bscan_cloud_.row_step);

  sensor_msgs::PointCloud2Iterator<float> bscan_iterX(bscan_cloud_, "x");
  sensor_msgs::PointCloud2Iterator<float> bscan_iterY(bscan_cloud_, "y");
  sensor_msgs::PointCloud2Iterator<float> bscan_iterZ(bscan_cloud_, "z");
  sensor_msgs::PointCloud2Iterator<float> bscan_iterAmps(bscan_cloud_,
                                                         "Amplitudes");

  gated_bscan_cloud_.header.stamp = obs_msg.header.stamp;
  gated_bscan_cloud_.header.frame_id = obs_msg.header.frame_id;
  gated_bscan_cloud_.width = obs_msg.ascan_length * obs_msg.num_ascans;
  gated_bscan_cloud_.row_step =
      gated_bscan_cloud_.point_step * gated_bscan_cloud_.width;
  gated_bscan_cloud_.data.clear();
  gated_bscan_cloud_.data.resize(gated_bscan_cloud_.row_step);

  sensor_msgs::PointCloud2Iterator<float> gated_bscan_iterX(gated_bscan_cloud_,
                                                            "x");
  sensor_msgs::PointCloud2Iterator<float> gated_bscan_iterY(gated_bscan_cloud_,
                                                            "y");
  sensor_msgs::PointCloud2Iterator<float> gated_bscan_iterZ(gated_bscan_cloud_,
                                                            "z");
  sensor_msgs::PointCloud2Iterator<float> gated_bscan_iterAmps(
      gated_bscan_cloud_, "Amplitudes");
  sensor_msgs::PointCloud2Iterator<float> gated_bscan_iterTof(
      gated_bscan_cloud_, "TimeofFlight");

  float dt = 1.0f / ((float)obs_msg.digitisation_rate * 1000000.0f); // sec
  // double time_in_wedge =       2.0 * obs_msg.wedge_depth / obs_msg.vel_wedge
  // / 1000.0;       // sec double time_in_couplant =    2.0 *
  // obs_msg.couplant_depth / obs_msg.vel_couplant / 1000.0; // sec double
  // time_in_specimen =    2.0 * obs_msg.specimen_depth / obs_msg.vel_material /
  // 1000.0; // sec

  float nan_value = std::numeric_limits<float>::quiet_NaN();

  float x;
  float y;
  float z;
  float normalised_amplitude;
  float gated_amplitude;
  float tof;

  int element_i = 0;

  for (const auto &ascan : obs_msg.ascans) {
    bool found_front_wall = false;
    float amp_front_wall = nan_value;
    float depth_front_wall = nan_value;
    bool found_back_wall = false;
    float amp_back_wall = nan_value;
    float depth_back_wall = nan_value;

    int i = 0;
    for (auto amplitude : ascan.amplitudes) {
      // GAT(S) --- GAT <Tn> <Gate Start> <Gate End>
      // Defines search gate start and end positions for the specified test.
      // By default, the gate units are in machine units.
      // A machine unit is defined by the digitisation rate (i.e. 10nSec for
      // 100MHz digitisation). Maybe assume 100 MHz to start with... double t =
      // (double)i * dt; double z = 0.0; if (t < time_in_wedge) {
      //     z = t * obs_msg.vel_wedge;
      // } else if (t < time_in_couplant) {
      //     z = (t - time_in_wedge) * obs_msg.vel_couplant
      //          + obs_msg.wedge_depth;
      // } else if (t < time_in_specimen) {
      //     z = (t - time_in_wedge - time_in_couplant) * obs_msg.vel_couplant
      //          + obs_msg.wedge_depth
      //          + obs_msg.couplant_depth;
      // }

      x = 0.0f;
      y = (float)((float)element_i * (float)obs_msg.element_pitch *
                  0.001f); // mm to m
      z = (float)((float)i * (float)obs_msg.vel_material * dt / 2.0f);

      if (use_tcg_ and z > (10.0f * 0.001f)) { // TODO: Param for skipping x mm
                                               // in before applying tcg
        float amplitude_tcg;

        // Amplify by n dB per l mm
        // amplitude_tcg = amplitude * 10.0 ^ (n * (z / l) / 20.0);
        amplitude_tcg =
            (float)amplitude *
            std::pow(10.0f, (amp_factor_ * (z / depth_factor_) / 20.0f));

        if (amplitude_tcg > (float)obs_msg.max_amplitude * tcg_limit_) {
          amplitude_tcg = (float)obs_msg.max_amplitude * tcg_limit_;
        } else if (amplitude_tcg < -(float)obs_msg.max_amplitude * tcg_limit_) {
          amplitude_tcg = -(float)obs_msg.max_amplitude * tcg_limit_;
        }

        amplitude = amplitude_tcg;
      }

      // Raw Amplitude
      // normalised_amplitude = (float)amplitude;

      // Normalised on Linear Scale
      normalised_amplitude = (float)amplitude / (float)obs_msg.max_amplitude;

      // Normalised on dB Scale
      // normalised_amplitude = 20.0 * (float)log10( (float)abs(
      // (float)amplitude / (float)obs_msg.max_amplitude) );

      *bscan_iterX = x;
      *bscan_iterY = y;
      *bscan_iterZ = z;
      *bscan_iterAmps = normalised_amplitude;

      ++bscan_iterX;
      ++bscan_iterY;
      ++bscan_iterZ;
      ++bscan_iterAmps;

      // Front wall gate
      if (!found_front_wall and normalised_amplitude > gate_front_wall_) {
        depth_front_wall = z;
        if (zero_to_front_wall_) {
          z = 0.0f;
        }
        amp_front_wall = normalised_amplitude;
        gated_amplitude = amp_front_wall;

        found_front_wall = true;
        if (!show_front_wall_) {
          x = nan_value;
          y = nan_value;
          z = nan_value;
          gated_amplitude = nan_value;
          tof = nan_value;
        }

        // Back wall gate
      } else if (found_front_wall and !found_back_wall and z < max_depth_ and
                 z > (depth_to_skip_ + depth_front_wall) and
                 normalised_amplitude > gate_back_wall_) {
        depth_back_wall = z;
        if (zero_to_front_wall_) {
          z = z - depth_front_wall;
        }
        amp_back_wall = normalised_amplitude;
        gated_amplitude = amp_back_wall;
        tof = depth_back_wall;

        found_back_wall = true;
      } else {
        x = nan_value;
        y = nan_value;
        z = nan_value;
        gated_amplitude = nan_value;
        tof = nan_value;
      }

      *gated_bscan_iterX = x;
      *gated_bscan_iterY = y;
      *gated_bscan_iterZ = z;
      *gated_bscan_iterAmps = gated_amplitude;
      *gated_bscan_iterTof = tof;

      ++gated_bscan_iterX;
      ++gated_bscan_iterY;
      ++gated_bscan_iterZ;
      ++gated_bscan_iterAmps;
      ++gated_bscan_iterTof;

      ++i;
    }
    ++element_i;
  }
}

void PeakNodelet::timerCb() {
  RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 600000,
                              "Node running");
  if (stream_) {
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 60000,
                                "Streaming data...");
    takeMeasurement();
  } else {
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 60000,
                                "Not streaming data...");
  }
}

} // namespace peak

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(peak::PeakNodelet)
