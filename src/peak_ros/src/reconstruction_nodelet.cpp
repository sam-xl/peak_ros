#include "peak_ros/reconstruction_nodelet.h"



namespace reconstruction_namespace {

ReconstructionNodelet::ReconstructionNodelet() : rclcpp::Node("reconstruction_node"),
    rate_(5),
    b_scan_count_(0),
    direction_(1)
{
    node_name_ = get_name();
    ns_ = get_namespace();

    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);    

    subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", 100, std::bind(&ReconstructionNodelet::callback, this, std::placeholders::_1));

    publisher_ =  this->create_publisher<sensor_msgs::msg::PointCloud2>("output", 10);

    publish_service_ = this->create_service<peak_ros::srv::StreamData>(ns_ + "/publish_volume", std::bind(&ReconstructionNodelet::publishSrvCb, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    ReconstructionNodelet::paramHandler("process_rate", rate_);
    
    timer_ = this->create_wall_timer(std::chrono::nanoseconds(1000000000 / rate_), std::bind(&ReconstructionNodelet::timerCb, this));

    ReconstructionNodelet::paramHandler("use_tf", use_tf_);
    ReconstructionNodelet::paramHandler("recon_frame_id", recon_frame_id_);
    ReconstructionNodelet::paramHandler("live_publish", live_publish_);
    if (!use_tf_) {
        ReconstructionNodelet::paramHandler("recon_const_vel", recon_const_vel_);
        ReconstructionNodelet::paramHandler("flip_direction", flip_direction_);
    }

    initialisePointcloud();
}


template <typename ParamType>
ParamType ReconstructionNodelet::paramHandler(std::string param_name, ParamType& param_value) {
    if (this->has_parameter(param_name))
    {
        param_value = this->get_parameter(param_name).get_value<ParamType>();
    }
    else
    {
        param_value =  this->declare_parameter(param_name, param_value);
    }

    RCLCPP_INFO_STREAM(this->get_logger(), node_name_ << ": Read in parameter " << param_name << " = " << param_value);

    return param_value;
}


void ReconstructionNodelet::initialisePointcloud() {
    point_cloud_.data.clear();

    int fields          = 5;
    int bytes_per_field = 4;

    point_cloud_.header.stamp = this->get_clock()->now();
    point_cloud_.header.frame_id = recon_frame_id_;
    sensor_msgs::PointCloud2Modifier modifier(point_cloud_);
    modifier.setPointCloud2Fields(
        fields,
        "x",              1, sensor_msgs::msg::PointField::FLOAT32,   // 32 bits = 4 bytes
        "y",              1, sensor_msgs::msg::PointField::FLOAT32,
        "z",              1, sensor_msgs::msg::PointField::FLOAT32,
        "Amplitudes",     1, sensor_msgs::msg::PointField::FLOAT32,
        "TimeofFlight", 1, sensor_msgs::msg::PointField::FLOAT32
        );

    point_cloud_.height = 1;
    point_cloud_.width = 0;
    point_cloud_.is_dense = true;
    point_cloud_.point_step = fields * bytes_per_field;
    point_cloud_.row_step = point_cloud_.point_step * point_cloud_.width;
    point_cloud_.data.resize(point_cloud_.row_step);
}


void ReconstructionNodelet::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    buffer_.push_back(*msg);
}


bool ReconstructionNodelet::publishSrvCb(const peak_ros::srv::StreamData::Request::SharedPtr request,
                                         peak_ros::srv::StreamData::Response::SharedPtr response) {
    RCLCPP_INFO_STREAM(this->get_logger(), node_name_ << 
        ": Publish UT volume request received: " << request->stream_data);

    if (request->stream_data) {
        publisher_->publish(point_cloud_);
        RCLCPP_INFO_STREAM(this->get_logger(), node_name_ << ": Published reconstruction");
        response->success = true;
        return true;
    } else {
        response->success = true;
        return true;
    }
}


void ReconstructionNodelet::timerCb() {
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 600000, node_name_ << ": Node running");

    if (!buffer_.empty()) {
        sensor_msgs::msg::PointCloud2* msg = &buffer_.front();
        sensor_msgs::msg::PointCloud2 output_pointcloud2;

        ////////////////////////////////////////////////////////////////////////////////////////////
        // Live full 3D reconstruction
        ////////////////////////////////////////////////////////////////////////////////////////////
        if (use_tf_) {
            try {
                trans_ = tfBuffer_->lookupTransform(recon_frame_id_,      // target frame
                                                   msg->header.frame_id, // source frame
                                                   msg->header.stamp,    // source time
                                                   rclcpp::Duration(std::chrono::seconds(3))    // time out
                                                   );

                tf2::doTransform<sensor_msgs::msg::PointCloud2>(*msg, output_pointcloud2, trans_);

                point_cloud_.width += output_pointcloud2.width;
                uint64_t prev_size = point_cloud_.data.size();
                point_cloud_.data.resize(point_cloud_.data.size() + output_pointcloud2.data.size());

                std::copy(
                    output_pointcloud2.data.begin(),
                    output_pointcloud2.data.end(),
                    point_cloud_.data.begin() + prev_size);

                point_cloud_.header.stamp = msg->header.stamp;

                buffer_.pop_front();

            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN_STREAM(this->get_logger(), node_name_ << 
                    ": Could not find transform " << recon_frame_id_ << 
                    " to " << msg->header.frame_id << 
                    ": " << ex.what());
            }

        ////////////////////////////////////////////////////////////////////////////////////////////
        // Post process psuedo 3D reconstruction for plotting
        ////////////////////////////////////////////////////////////////////////////////////////////
        } else {
            trans_.header.stamp = msg->header.stamp;
            trans_.header.frame_id = recon_frame_id_;
            trans_.child_frame_id = msg->header.frame_id;


            if (b_scan_count_ == 0) {
                trans_.transform.translation.x = 0.0l;
                // trans_.transform.translation.x = 0.0751l;
                trans_.transform.translation.y = 0.0l;
                trans_.transform.translation.z = 0.0l;
                trans_.transform.rotation.x =  0.0l;
                trans_.transform.rotation.y = -1.0l;
                trans_.transform.rotation.z =  0.0l;
                trans_.transform.rotation.w =  0.0l;

            } else {
                rclcpp::Duration dt = rclcpp::Time(msg->header.stamp) - prev_observation_time_;

                // During scan pass
                if (dt.seconds() < 4.0l) {
                    if (direction_ == 1) {
                        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 30000, node_name_ << ": Going forwards");
                        trans_.transform.translation.x += recon_const_vel_ * dt.seconds();
                    } else {
                        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 30000, node_name_ << ": Going backwards");
                        trans_.transform.translation.x -= recon_const_vel_ * dt.seconds();
                    }
                // Switching raster paths
                } else {
                    publisher_->publish(point_cloud_);
                    RCLCPP_INFO_STREAM(this->get_logger(), node_name_ << ": Published reconstruction");
                    // initialisePointcloud();
                    if (flip_direction_) {
                        if (direction_ == 1) {
                            RCLCPP_INFO_STREAM(this->get_logger(), node_name_ << ": Changing direction");
                            direction_ = -1;
                            trans_.transform.translation.y += 0.09l;
                            trans_.transform.rotation.x =  1.0l;
                            trans_.transform.rotation.y =  0.0l;
                            trans_.transform.rotation.z =  0.0l;
                            trans_.transform.rotation.w =  0.0l;
                        } else {
                            RCLCPP_INFO_STREAM(this->get_logger(), node_name_ << ": Changing direction");
                            direction_ = 1;
                            trans_.transform.rotation.x =  0.0l;
                            trans_.transform.rotation.y = -1.0l;
                            trans_.transform.rotation.z =  0.0l;
                            trans_.transform.rotation.w =  0.0l;
                        }
                    } else {
                        trans_.transform.translation.x  = 0.0l;
                        trans_.transform.translation.y += 0.045l;
                        RCLCPP_INFO_STREAM(this->get_logger(), node_name_ << ": Reset start of pass to zero: " << trans_.transform.translation.x);
                    }
                }
            }

            tf2::doTransform<sensor_msgs::msg::PointCloud2>(*msg, output_pointcloud2, trans_);

            // RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 10000, node_name_ << ": transform translation x: " << trans_.transform.translation.x);
            // RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 10000, node_name_ << ": transform translation y: " << trans_.transform.translation.y);
            // RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 10000, node_name_ << ": transform translation z: " << trans_.transform.translation.z);

            point_cloud_.width += output_pointcloud2.width;
            uint64_t prev_size = point_cloud_.data.size();
            point_cloud_.data.resize(point_cloud_.data.size() + output_pointcloud2.data.size());

            std::copy(
                output_pointcloud2.data.begin(),
                output_pointcloud2.data.end(),
                point_cloud_.data.begin() + prev_size);

            point_cloud_.header.stamp = msg->header.stamp;

            buffer_.pop_front();

            prev_observation_time_ = msg->header.stamp;
            b_scan_count_++;
        }

        if (live_publish_) {
            publisher_->publish(point_cloud_);
        }
    }
}


} // namespace reconstruction_namespace

