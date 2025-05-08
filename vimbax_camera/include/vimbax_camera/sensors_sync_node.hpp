#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <deque>
#include <memory>
#include <mutex>

#include "vimbax_camera/camera_interface.hpp"

namespace vimbax_camera
{

class SensorsSyncNode : public rclcpp::Node, public std::enable_shared_from_this<SensorsSyncNode>
{
public:
  SensorsSyncNode(const rclcpp::NodeOptions & options);
  ~SensorsSyncNode();

  bool initialize();

private:
  using ImageMsg = sensor_msgs::msg::Image;
  using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
  using ImuMsg = sensor_msgs::msg::Imu;
  using CameraInterface = vimbax_camera_sync::CameraInterface;
  using CameraFrame = vimbax_camera_sync::CameraFrame;

  // Callbacks
  void imu_callback(const ImuMsg::SharedPtr msg);
  void left_frame_callback(const CameraFrame & frame);
  void right_frame_callback(const CameraFrame & frame);

  // Trigger control
  void trigger_pwm();
  void disable_pwm();

  rclcpp::Time predict_stamp_using_imu_buffer(
    uint64_t frame_id,
    uint64_t dt_left,
    uint64_t dt_right);

  // Timestamp sync logic
  void sync_and_publish_frames();

  // Helper methods
  void trim_old_data();
  std::pair<CameraFrame*, CameraFrame*> find_earliest_stereo_pair();

  // Camera interface
  std::unique_ptr<CameraInterface> left_camera_;
  std::unique_ptr<CameraInterface> right_camera_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> left_info_mgr_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> right_info_mgr_;
  
  // Params
  std::string imu_topic_;
  std::string left_camera_id_;
  std::string right_camera_id_;
  std::string left_camera_link_;
  std::string right_camera_link_;
  std::string left_camera_info_url_;
  std::string right_camera_info_url_;
  int pwm_freq_;
  int pwm_divider_;
  int pwm_duty_;
  int buffer_count_;
  bool sync_first_sample_only_ = true;
  std::string warning_topic_;
  double camera_buffer_duration_;
  double imu_buffer_duration_;  
  bool print_frames_data_ = false;
  bool print_stereo_pair_data_ = false;

  // Buffers
  std::deque<CameraFrame> left_buffer_;
  std::deque<CameraFrame> right_buffer_;
  std::deque<std::pair<size_t, ImuMsg::SharedPtr>> imu_buffer_;
  std::mutex buffer_mutex_;

  // Frame ID tracking
  uint64_t last_synced_frame_id_ = 0;
  
  size_t imu_index_ = 0;

  // Time sync references
  bool camera_time_initialized_ = false;
  uint64_t camera_start_ts_left_ = 0;
  uint64_t camera_start_ts_right_ = 0;
  rclcpp::Time imu_start_time_;  
  bool imu_first_call_ = true;

  // Publishers
  rclcpp::Publisher<ImageMsg>::SharedPtr left_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr right_pub_;
  rclcpp::Publisher<CameraInfoMsg>::SharedPtr left_info_pub_;
  rclcpp::Publisher<CameraInfoMsg>::SharedPtr right_info_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr warning_pub_;
  rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;
};

}  // namespace vimbax_camera
