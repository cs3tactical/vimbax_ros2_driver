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
  buffer_count_ = this->declare_parameter<int>("buffer_count", 10);

  left_info_mgr_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, left_camera_id_, left_camera_info_url_);
  right_info_mgr_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, right_camera_id_, right_camera_info_url_);

  left_pub_ = this->create_publisher<ImageMsg>("camera_left/image_raw", 10);
  right_pub_ = this->create_publisher<ImageMsg>("camera_right/image_raw", 10);
  left_info_pub_ = this->create_publisher<CameraInfoMsg>("camera_left/camera_info", 10);
  right_info_pub_ = this->create_publisher<CameraInfoMsg>("camera_right/camera_info", 10);
}

bool SensorsSyncNode::initialize()
{
  auto self_node = std::static_pointer_cast<rclcpp::Node>(
    std::enable_shared_from_this<SensorsSyncNode>::shared_from_this());
  
  left_camera_ = std::make_unique<CameraInterface>(self_node, left_camera_id_);
  right_camera_ = std::make_unique<CameraInterface>(self_node, right_camera_id_);

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
  if (imu_buffer_.size() > 100) imu_buffer_.pop_front();
  sync_and_publish_frames();
}

void SensorsSyncNode::left_frame_callback(const CameraFrame & frame)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  left_buffer_.push_back(frame);
  if (left_buffer_.size() > 15) left_buffer_.pop_front();
  sync_and_publish_frames();
}

void SensorsSyncNode::right_frame_callback(const CameraFrame & frame)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  right_buffer_.push_back(frame);
  if (right_buffer_.size() > 15) right_buffer_.pop_front();
  sync_and_publish_frames();
}

void SensorsSyncNode::sync_and_publish_frames()
{
  while (!left_buffer_.empty() && !right_buffer_.empty() && !imu_buffer_.empty()) {
    auto & l = left_buffer_.front();
    auto & r = right_buffer_.front();

    if (l.frame_id != r.frame_id) {
      if (l.frame_id < r.frame_id)
        left_buffer_.pop_front();
      else
        right_buffer_.pop_front();
      continue;
    }

    uint64_t expected_frame = last_synced_frame_id_ + 1;
    if (l.frame_id != expected_frame) {
      break;
    }

    size_t imu_index = expected_frame * pwm_divider_;
    if (imu_index >= imu_buffer_.size()) {
      break;
    }

    auto imu_msg = imu_buffer_[imu_index];
    l.image.header.stamp = imu_msg->header.stamp;
    r.image.header.stamp = imu_msg->header.stamp;

    auto left_info = left_info_mgr_->getCameraInfo();
    auto right_info = right_info_mgr_->getCameraInfo();
    left_info.header = l.image.header;
    right_info.header = r.image.header;

    left_pub_->publish(l.image);
    right_pub_->publish(r.image);
    left_info_pub_->publish(left_info);
    right_info_pub_->publish(right_info);

    last_synced_frame_id_ = l.frame_id;
    last_imu_time_ = imu_msg->header.stamp;

    left_buffer_.pop_front();
    right_buffer_.pop_front();
  }
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
