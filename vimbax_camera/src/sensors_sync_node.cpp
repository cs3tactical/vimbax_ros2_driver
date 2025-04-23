#include "vimbax_camera/sensors_sync_node.hpp"

namespace vimbax_camera
{

using CameraInterface = vimbax_camera_sync::CameraInterface;
using CameraFrame = vimbax_camera_sync::CameraFrame;

SensorsSyncNode::SensorsSyncNode(const rclcpp::NodeOptions & options)
: Node("sensors_sync_node", options)
{
  left_camera_id_ = this->declare_parameter<std::string>("left_camera_id");
  right_camera_id_ = this->declare_parameter<std::string>("right_camera_id");
  imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu/data");
  left_camera_info_url_ = this->declare_parameter<std::string>("left_camera_info_url");
  right_camera_info_url_ = this->declare_parameter<std::string>("right_camera_info_url");
  pwm_freq_ = this->declare_parameter<int>("pwm_frequency", 105);
  pwm_divider_ = this->declare_parameter<int>("pwm_divider", 7);
  pwm_duty_ = this->declare_parameter<int>("pwm_duty", 50);
  warning_topic_ = this->declare_parameter<std::string>("warning_topic", "/diagnostics/stereo_warnings");
  camera_buffer_duration_ = this->declare_parameter<double>("camera_buffer_duration", 3.0);
  imu_buffer_duration_ = this->declare_parameter<double>("imu_buffer_duration", 3.0);

  left_info_mgr_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, left_camera_id_, left_camera_info_url_);
  right_info_mgr_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, right_camera_id_, right_camera_info_url_);

  left_pub_ = this->create_publisher<ImageMsg>("camera_left/image_raw", 10);
  right_pub_ = this->create_publisher<ImageMsg>("camera_right/image_raw", 10);
  left_info_pub_ = this->create_publisher<CameraInfoMsg>("camera_left/camera_info", 10);
  right_info_pub_ = this->create_publisher<CameraInfoMsg>("camera_right/camera_info", 10);
  warning_pub_ = this->create_publisher<std_msgs::msg::String>(warning_topic_, 10);
}

bool SensorsSyncNode::initialize()
{
  left_camera_ = std::make_unique<CameraInterface>(this, left_camera_id_);
  right_camera_ = std::make_unique<CameraInterface>(this, right_camera_id_);

  if (!left_camera_->initialize(
          [this](const CameraFrame & f) { this->left_frame_callback(f); })) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize left camera");
    return false;
  }

  if (!right_camera_->initialize(
          [this](const CameraFrame & f) { this->right_frame_callback(f); })) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize right camera");
    return false;
  }

  imu_sub_ = this->create_subscription<ImuMsg>(
    imu_topic_, 100,
    std::bind(&SensorsSyncNode::imu_callback, this, std::placeholders::_1));

  trigger_pwm();
  return true;
}

SensorsSyncNode::~SensorsSyncNode()
{
  disable_pwm();
}

void SensorsSyncNode::trigger_pwm()
{
  std::string command = "sudo /usr/local/bin/pwm_control.sh 0 " +
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
  std::string command = "sudo /usr/local/bin/pwm_control.sh 0 disable";
  int ret = std::system(command.c_str());
  if (ret == 0) {
    RCLCPP_INFO(this->get_logger(), "PWM signal disabled.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to disable PWM signal.");
  }
}

void SensorsSyncNode::imu_callback(const ImuMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  imu_buffer_.push_back(msg);
  imu_msg_count_++;
  // RCLCPP_INFO(this->get_logger(),
  //   "Received IMU message [%zu] - timestamp: %.6f",
  //   imu_msg_count_,
  //   msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
  sync_and_publish_frames();
}

void SensorsSyncNode::left_frame_callback(const CameraFrame & frame)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  left_buffer_.push_back(frame);
  // RCLCPP_INFO(this->get_logger(),
  //   "Received frame from LEFT camera - frame_id: %lu, timestamp: %.6f",
  //   frame.frame_id,
  //   frame.internal_timestamp_ns / 1e9);
  sync_and_publish_frames();
}

void SensorsSyncNode::right_frame_callback(const CameraFrame & frame)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  right_buffer_.push_back(frame);
  // RCLCPP_INFO(this->get_logger(),
  //   "Received frame from RIGHT camera - frame_id: %lu, timestamp: %.6f",
  //   frame.frame_id,
  //   frame.internal_timestamp_ns / 1e9);
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
  trim_old(imu_buffer_, [](const ImuMsg::SharedPtr & msg) {
    return msg->header.stamp;
  }, imu_buffer_duration_);
}

std::pair<CameraFrame*, CameraFrame*> SensorsSyncNode::find_earliest_stereo_pair()
{
  RCLCPP_INFO(this->get_logger(), "trying to find stereo pair");

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

void SensorsSyncNode::sync_and_publish_frames()
{
  // trim_old_data();
  if (left_buffer_.empty() || right_buffer_.empty()) {
    // RCLCPP_INFO(this->get_logger(), "cameras buffer empty");
    return;
  }

  auto [left, right] = find_earliest_stereo_pair();
  if (!left || !right) {
    auto warn = std_msgs::msg::String();
    warn.data = "[SyncNode] No matching stereo pair found.";
    warning_pub_->publish(warn);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "[SyncNode] Found matching stereo pair with frame_id: %lu", left->frame_id);

  if (left->frame_id == 0 && !camera_time_initialized_) {
    camera_start_ts_left_ = left->internal_timestamp_ns;
    camera_start_ts_right_ = right->internal_timestamp_ns;
    imu_start_time_ = imu_buffer_.front()->header.stamp;
    camera_time_initialized_ = true;
  }

  if (!camera_time_initialized_) return;

  uint64_t dt_left = left->internal_timestamp_ns - camera_start_ts_left_;
  uint64_t dt_right = right->internal_timestamp_ns - camera_start_ts_right_;
  uint64_t avg_offset_ns = (dt_left + dt_right) / 2;
  rclcpp::Time predicted_stamp = imu_start_time_ + rclcpp::Duration::from_nanoseconds(avg_offset_ns);

  rclcpp::Time best_imu_stamp = predicted_stamp;
  rclcpp::Duration min_diff = rclcpp::Duration::from_seconds(1.0);
  for (auto & imu : imu_buffer_) {
    rclcpp::Time imu_time = imu->header.stamp;
    auto diff = std::abs((imu_time - predicted_stamp).nanoseconds());
    if (diff < min_diff.nanoseconds()) {
      best_imu_stamp = imu_time;
      min_diff = rclcpp::Duration::from_nanoseconds(diff);
    }
  }

  rclcpp::Time final_stamp = min_diff.nanoseconds() < 2000000 ? best_imu_stamp : predicted_stamp;

  left->image.header.stamp = final_stamp;
  right->image.header.stamp = final_stamp;

  auto left_info = left_info_mgr_->getCameraInfo();
  auto right_info = right_info_mgr_->getCameraInfo();
  left_info.header.stamp = final_stamp;
  right_info.header.stamp = final_stamp;

  left_pub_->publish(left->image);
  right_pub_->publish(right->image);
  left_info_pub_->publish(left_info);
  right_info_pub_->publish(right_info);

  last_synced_frame_id_ = left->frame_id;

  left_buffer_.erase(std::remove_if(left_buffer_.begin(), left_buffer_.end(),
    [&](const CameraFrame & f) { return f.frame_id <= last_synced_frame_id_; }),
    left_buffer_.end());

  right_buffer_.erase(std::remove_if(right_buffer_.begin(), right_buffer_.end(),
    [&](const CameraFrame & f) { return f.frame_id <= last_synced_frame_id_; }),
    right_buffer_.end());
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
