#include "vimbax_camera/sensors_sync_node.hpp"

namespace vimbax_camera
{

using CameraInterface = vimbax_camera_sync::CameraInterface;
using CameraFrame = vimbax_camera_sync::CameraFrame;

SensorsSyncNode::SensorsSyncNode(const rclcpp::NodeOptions & options)
: Node("sensors_sync_node", options)
{
  // imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu/data");
  left_camera_id_ = this->declare_parameter<std::string>("left_camera_id");
  right_camera_id_ = this->declare_parameter<std::string>("right_camera_id");
  left_camera_link_ = this->declare_parameter<std::string>("left_camera_link", "camera_left");
  right_camera_link_ = this->declare_parameter<std::string>("right_camera_link", "camera_right");
  left_camera_info_url_ = this->declare_parameter<std::string>("left_camera_info_url");
  right_camera_info_url_ = this->declare_parameter<std::string>("right_camera_info_url");

  pwm_freq_ = this->declare_parameter<int>("pwm_frequency", 100);
  pwm_divider_ = this->declare_parameter<int>("pwm_divider", 7);
  pwm_duty_ = this->declare_parameter<int>("pwm_duty", 50);
  // sync_first_sample_only_ = this->declare_parameter<bool>("sync_first_sample_only", true);
  warning_topic_ = this->declare_parameter<std::string>("warning_topic", "/diagnostics/stereo_warnings");
  camera_buffer_duration_ = this->declare_parameter<double>("camera_buffer_duration", 3.0);
  // imu_buffer_duration_ = this->declare_parameter<double>("imu_buffer_duration", 3.0);
  print_frames_data_ = this->declare_parameter<bool>("print_frames_data", false);
  print_stereo_pair_data_ = this->declare_parameter<bool>("print_stereo_pair_data", false);

  left_info_mgr_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, left_camera_link_, left_camera_info_url_);
  right_info_mgr_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, right_camera_link_, right_camera_info_url_);

  left_pub_ = this->create_publisher<ImageMsg>("camera_left/image_raw", 10);
  right_pub_ = this->create_publisher<ImageMsg>("camera_right/image_raw", 10);
  left_info_pub_ = this->create_publisher<CameraInfoMsg>("camera_left/camera_info", 10);
  right_info_pub_ = this->create_publisher<CameraInfoMsg>("camera_right/camera_info", 10);
  left_metadata_pub_ = this->create_publisher<vimbax_camera_msgs::msg::CameraMetadata>("camera_left/metadata", 10);
  right_metadata_pub_ = this->create_publisher<vimbax_camera_msgs::msg::CameraMetadata>("camera_right/metadata", 10);
  left_temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("camera_left/temperature", 10);
  right_temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("camera_right/temperature", 10);
  
  warning_pub_ = this->create_publisher<std_msgs::msg::String>(warning_topic_, 10);
}

bool SensorsSyncNode::initialize()
{
  // Safely initialize VimbaX API once on main thread
  RCLCPP_INFO(this->get_logger(), "Loading Vimbax API ...");
  auto api = vimbax_camera::VmbCAPI::get_instance();
  if (!api) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load VimbaX API (VmbStartup failed).");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Vimbax API loaded");

  disable_pwm();

  // Parallel camera initialization
  std::atomic<bool> right_ok{false}, left_ok{false};
  std::mutex err_mutex;
  std::string err_msg;

  std::thread right_thread([&] {
    RCLCPP_INFO(this->get_logger(), "Loading right camera ...");
    right_camera_ = std::make_unique<CameraInterface>(this, right_camera_id_);
    if (right_camera_->initialize([this](const CameraFrame & f) { this->right_frame_callback(f); })) {
      right_ok = true;
    } else {
      std::lock_guard<std::mutex> lock(err_mutex);
      err_msg += "Right camera failed to initialize.\n";
    }
  });

  std::thread left_thread([&] {
    RCLCPP_INFO(this->get_logger(), "Loading left camera ...");
    left_camera_ = std::make_unique<CameraInterface>(this, left_camera_id_);
    if (left_camera_->initialize([this](const CameraFrame & f) { this->left_frame_callback(f); })) {
      left_ok = true;
    } else {
      std::lock_guard<std::mutex> lock(err_mutex);
      err_msg += "Left camera failed to initialize.\n";
    }
  });

  right_thread.join();
  left_thread.join();

  if (!right_ok || !left_ok) {
    RCLCPP_FATAL(this->get_logger(), "Camera initialization failed:\n%s", err_msg.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "initialized cameras successfully");

  // IMU subscription disabled - timestamps are anchored from first camera frame only
  // if (!sync_first_sample_only_) {
  //   imu_sub_ = this->create_subscription<ImuMsg>(
  //     imu_topic_, 100,
  //     std::bind(&SensorsSyncNode::imu_callback, this, std::placeholders::_1));
  //   RCLCPP_INFO(this->get_logger(), "subscribed to IMU topic for buffer-based stamp refinement");
  // }

  trigger_pwm();
  return true;
}


SensorsSyncNode::~SensorsSyncNode()
{
  disable_pwm();
}

void SensorsSyncNode::trigger_pwm()
{
  std::string command = "/usr/local/bin/pwm_control.sh 0 " +
                        std::to_string(pwm_freq_) + " " + std::to_string(pwm_duty_);
  int ret = std::system(command.c_str());
  if (ret == 0) {
    RCLCPP_INFO(this->get_logger(), "PWM signal triggered at %d Hz", pwm_freq_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to trigger PWM signal.");
  }
}

void SensorsSyncNode::disable_pwm()
{
  std::string command = "/usr/local/bin/pwm_control.sh 0 disable";
  int ret = std::system(command.c_str());
  if (ret == 0) {
    RCLCPP_INFO(this->get_logger(), "PWM signal disabled.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to disable PWM signal.");
  }
}

// void SensorsSyncNode::imu_callback(const ImuMsg::SharedPtr msg)
// {
//   std::lock_guard<std::mutex> lock(buffer_mutex_);
//   imu_buffer_.emplace_back(imu_index_, msg);
//   RCLCPP_INFO(this->get_logger(),
//     "Received IMU message [%zu] - timestamp: %.9f",
//     imu_index_,
//     msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
//   imu_index_++;
// }

void SensorsSyncNode::left_frame_callback(const CameraFrame & frame)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);

  if (frame.frame_id) {  // Ignore frame ID #0 - the camera sends it before trigger
    left_buffer_.push_back(frame);
  }

  auto diff = frame.frame_id - last_left_frame_id_;
  if (diff > 1) {
    std_msgs::msg::String msg;
    msg.data = "[LEFT] " + std::to_string(diff - 1) + 
               " frames dropped - didn\'t received from camera (last: " + std::to_string(last_left_frame_id_) +
               ", current: " + std::to_string(frame.frame_id) + ")";
    warning_pub_->publish(msg);
  }
  last_left_frame_id_ = frame.frame_id;

  if (print_frames_data_)
  {
    RCLCPP_INFO(this->get_logger(),
      "Received frame from LEFT camera - frame_id: %lu, timestamp: %.6f",
      frame.frame_id,
      frame.internal_timestamp_ns / 1e9);    
  }

  sync_and_publish_frames();
}

void SensorsSyncNode::right_frame_callback(const CameraFrame & frame)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);

  if (frame.frame_id) {  // Ignore frame ID #0 - the camera sends it before trigger
    right_buffer_.push_back(frame);
  }

  auto diff = frame.frame_id - last_right_frame_id_;
  if (diff > 1) {
    std_msgs::msg::String msg;
    msg.data = "[RIGHT] " + std::to_string(diff - 1) + 
               " frames dropped - didn\'t received from camera (last: " + std::to_string(last_right_frame_id_) +
               ", current: " + std::to_string(frame.frame_id) + ")";
    warning_pub_->publish(msg);
  }
  last_right_frame_id_ = frame.frame_id;

  if (print_frames_data_)
  {
    RCLCPP_INFO(this->get_logger(),
      "Received frame from RIGHT camera - frame_id: %lu, timestamp: %.6f",
      frame.frame_id,
      frame.internal_timestamp_ns / 1e9);
  }
  sync_and_publish_frames();
}

void SensorsSyncNode::trim_old_data()
{
  auto now = this->now();
  auto trim_old = [&](auto & buffer, auto get_time, double threshold) {
    while (!buffer.empty() && (now - get_time(buffer.front())).seconds() > threshold) {
      buffer.pop_front();
    }
  };

  trim_old(left_buffer_, [](const CameraFrame & f) {
    return rclcpp::Time(f.image.header.stamp);
  }, camera_buffer_duration_);
  trim_old(right_buffer_, [](const CameraFrame & f) {
    return rclcpp::Time(f.image.header.stamp);
  }, camera_buffer_duration_);
  // trim_old(imu_buffer_, [](const std::pair<uint64_t, ImuMsg::SharedPtr> & pair) {
  //   return pair.second->header.stamp;
  // }, imu_buffer_duration_);
}

std::pair<CameraFrame*, CameraFrame*> SensorsSyncNode::find_earliest_stereo_pair()
{
  while (!left_buffer_.empty() && !right_buffer_.empty()) {
    CameraFrame & l = left_buffer_.front();
    CameraFrame & r = right_buffer_.front();

    if (l.frame_id == r.frame_id) {
      return {&l, &r};
    } else if (l.frame_id < r.frame_id) {
      left_buffer_.pop_front();
    } else {
      right_buffer_.pop_front();
    }
  }
  return {nullptr, nullptr};
}

// rclcpp::Time SensorsSyncNode::predict_stamp_using_imu_buffer(
//   uint64_t frame_id,
//   uint64_t dt_left,
//   uint64_t dt_right)
// {
//   uint64_t avg_offset_ns = (dt_left + dt_right) / 2;
//   rclcpp::Time predicted_stamp = camera_start_ros_time_ + rclcpp::Duration::from_nanoseconds(avg_offset_ns);
//
//   size_t expected_imu_index = frame_id * pwm_divider_;
//   rclcpp::Time best_imu_stamp = predicted_stamp;
//   rclcpp::Duration min_diff = rclcpp::Duration::from_seconds(0.1);
//
//   for (const auto & [idx, imu] : imu_buffer_) {
//     if (idx < expected_imu_index) continue;
//     rclcpp::Time imu_time = imu->header.stamp;
//     auto diff = std::abs((imu_time - predicted_stamp).nanoseconds());
//     if (diff < min_diff.nanoseconds()) {
//       best_imu_stamp = imu_time;
//       min_diff = rclcpp::Duration::from_nanoseconds(diff);
//     }
//   }
//
//   if (min_diff.nanoseconds() < 2000000) {  // within 2ms
//     return best_imu_stamp;
//   } else {
//     return predicted_stamp;
//   }
// }


void SensorsSyncNode::sync_and_publish_frames()
{
  // trim_old_data();
  if (left_buffer_.empty() || right_buffer_.empty()) {
    return;
  }

  auto [left, right] = find_earliest_stereo_pair();
  if (!left || !right) {
    auto warn = std_msgs::msg::String();
    warn.data = "[sync_and_publish_frames] No matching stereo pair found.";
    warning_pub_->publish(warn);
    return;
  }

  // Start timestamp sync from frame ID #1, not 0:
  if (left->frame_id == 1 && !camera_time_initialized_) {
    camera_start_ts_left_ = left->internal_timestamp_ns;
    camera_start_ts_right_ = right->internal_timestamp_ns;
    camera_start_ros_time_ = this->now();
    camera_time_initialized_ = true;

    RCLCPP_INFO(this->get_logger(),
      "Camera start time anchored from first stereo pair (frame #1): %.9f",
      camera_start_ros_time_.seconds());
  }

  if (!camera_time_initialized_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Camera start time not set yet. Waiting for first stereo pair.");
    return;
  }

  // if (!camera_time_initialized_) return;

  uint64_t dt_left = left->internal_timestamp_ns - camera_start_ts_left_;
  uint64_t dt_right = right->internal_timestamp_ns - camera_start_ts_right_;

  uint64_t avg_offset_ns = (dt_left + dt_right) / 2;
  rclcpp::Time final_stamp = camera_start_ros_time_ + rclcpp::Duration::from_nanoseconds(avg_offset_ns);

  // IMU-based stamp refinement disabled:
  // final_stamp = predict_stamp_using_imu_buffer(left->frame_id, dt_left, dt_right);

  if (print_stereo_pair_data_)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Stereo pair %lu - timestamp: %.9f, left_ts: %.9f s, right_ts: %.9f s",
      left->frame_id,
      final_stamp.seconds(),
      static_cast<double>(left->internal_timestamp_ns) / 1e9,
      static_cast<double>(right->internal_timestamp_ns) / 1e9
    );      
  }

  left->image.header.stamp = final_stamp;
  left->image.header.frame_id = left_camera_link_;
  right->image.header.stamp = final_stamp;
  right->image.header.frame_id = right_camera_link_;

  auto left_info = left_info_mgr_->getCameraInfo();
  auto right_info = right_info_mgr_->getCameraInfo();
  left_info.header.stamp = final_stamp;
  left_info.header.frame_id = left_camera_link_;
  right_info.header.stamp = final_stamp;
  right_info.header.frame_id = right_camera_link_;

  left_pub_->publish(left->image);
  right_pub_->publish(right->image);
  left_info_pub_->publish(left_info);
  right_info_pub_->publish(right_info);

  vimbax_camera_msgs::msg::CameraMetadata left_metadata = build_metadata_msg_body(left->metadata);
  left_metadata.header.stamp = final_stamp;
  left_metadata.header.frame_id = left_camera_link_;
  left_metadata.frame_index = left->frame_id;
  left_metadata.internal_timestamp = rclcpp::Time(left->internal_timestamp_ns);
  
  vimbax_camera_msgs::msg::CameraMetadata right_metadata = build_metadata_msg_body(right->metadata);
  right_metadata.header.stamp = final_stamp;
  right_metadata.header.frame_id = right_camera_link_;
  right_metadata.frame_index = right->frame_id;
  right_metadata.internal_timestamp = rclcpp::Time(right->internal_timestamp_ns);  

  left_metadata_pub_->publish(left_metadata);
  right_metadata_pub_->publish(right_metadata);

  if (left->frame_id % 10 == 0) {
    // Publish left camera temperature
    sensor_msgs::msg::Temperature left_temp_msg;
    left_temp_msg.header.stamp = final_stamp;
    left_temp_msg.header.frame_id = left_camera_link_;
    left_temp_msg.temperature = left->metadata.device_temp_c;
    left_temp_msg.variance = 0.0;
    left_temp_pub_->publish(left_temp_msg);
  
    // Publish right camera temperature
    sensor_msgs::msg::Temperature right_temp_msg;
    right_temp_msg.header.stamp = final_stamp;
    right_temp_msg.header.frame_id = right_camera_link_;
    right_temp_msg.temperature = right->metadata.device_temp_c;
    right_temp_msg.variance = 0.0;
    right_temp_pub_->publish(right_temp_msg);
  }

  last_synced_frame_id_ = left->frame_id;

  left_buffer_.erase(std::remove_if(left_buffer_.begin(), left_buffer_.end(),
    [&](const CameraFrame & f) { return f.frame_id <= last_synced_frame_id_; }),
    left_buffer_.end());

  right_buffer_.erase(std::remove_if(right_buffer_.begin(), right_buffer_.end(),
    [&](const CameraFrame & f) { return f.frame_id <= last_synced_frame_id_; }),
    right_buffer_.end());
}

vimbax_camera_msgs::msg::CameraMetadata SensorsSyncNode::build_metadata_msg_body(
  const vimbax_camera_sync::CameraMetadata & meta)
{
  vimbax_camera_msgs::msg::CameraMetadata msg;

  msg.exposure_time_us = meta.exposure_time_us;

  msg.device_temp_c = meta.device_temp_c;
  msg.temp_status = meta.temp_status;

  msg.gain_db = meta.gain_db;

  return msg;
}

}  // namespace vimbax_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vimbax_camera::SensorsSyncNode>(rclcpp::NodeOptions{});
  if (!node->initialize()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
