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
  // Stream recovery. A frame corrupted on the CSI link costs the driver one capture buffer until
  // the stream restarts, so the camera first loses frames the other one delivers, then stops.
  // Restart both streams when no pair was published for stall_timeout s, or when
  // max_unmatched_frames frames arrived from only one camera within unmatched_window s (0 disables).
  stall_timeout_ = std::chrono::duration<double>(
    this->declare_parameter<double>("stall_timeout", 1.0));
  max_unmatched_frames_ = static_cast<size_t>(
    this->declare_parameter<int>("max_unmatched_frames", 3));
  unmatched_window_ = std::chrono::duration<double>(
    this->declare_parameter<double>("unmatched_window", 5.0));

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

  // Created here rather than in initialize(): a SIGINT during the camera open invalidates the
  // context, and creating a timer then throws. It only fires once spinning, after initialize().
  watchdog_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200), std::bind(&SensorsSyncNode::watchdog_callback, this));
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
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    last_pair_time_ = std::chrono::steady_clock::now();
  }
  return true;
}


SensorsSyncNode::~SensorsSyncNode()
{
  disable_pwm();
}

void SensorsSyncNode::shutdown()
{
  // Called before rclcpp::shutdown(): closing the cameras while rclcpp's signal handlers are
  // still installed means launch's follow-up SIGINT/SIGTERM can't kill the process with the
  // streams open (the kernel then takes ~16 s to release them).
  disable_pwm();
  left_camera_.reset();
  right_camera_.reset();
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

void SensorsSyncNode::watchdog_callback()
{
  std::string reason;
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    const auto now = std::chrono::steady_clock::now();
    while (!unmatched_frame_times_.empty() &&
           now - unmatched_frame_times_.front() > unmatched_window_) {
      unmatched_frame_times_.pop_front();
    }

    const auto since_last_pair =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_pair_time_);
    if (stall_timeout_.count() > 0 && since_last_pair > stall_timeout_) {
      reason = "no stereo pair for " + std::to_string(since_last_pair.count()) + " ms";
    } else if (max_unmatched_frames_ > 0 && unmatched_frame_times_.size() >= max_unmatched_frames_) {
      reason = std::to_string(unmatched_frame_times_.size()) + " frames arrived from only one camera";
    }
  }

  if (reason.empty()) {
    return;
  }

  // Restart without buffer_mutex_ held: stopping a stream waits for its frame callback to return
  ++restart_count_;
  std_msgs::msg::String msg;
  msg.data = "Restarting camera streams (#" + std::to_string(restart_count_) + "): " + reason;
  RCLCPP_WARN(this->get_logger(), "%s", msg.data.c_str());
  warning_pub_->publish(msg);

  if (!restart_streams()) {
    // Exit with an error so the service restarts the whole node
    RCLCPP_FATAL(this->get_logger(), "Failed to restart camera streams, exiting.");
    fatal_error_ = true;
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Camera streams restarted.");
}

bool SensorsSyncNode::restart_streams()
{
  // As at startup, no trigger while the streams restart, so both cameras count frames from the
  // same first trigger and their frame IDs pair up again
  disable_pwm();

  auto on_both_cameras = [this](const std::function<bool(CameraInterface &)> & action) {
    bool left_ok = false;
    std::thread left_thread([&] { left_ok = action(*left_camera_); });
    bool right_ok = action(*right_camera_);
    left_thread.join();
    return left_ok && right_ok;
  };

  if (!on_both_cameras([](CameraInterface & camera) { return camera.stop_streaming(); })) {
    return false;
  }
  reset_sync_state();
  if (!on_both_cameras([](CameraInterface & camera) { return camera.start_streaming(); })) {
    return false;
  }

  trigger_pwm();
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  last_pair_time_ = std::chrono::steady_clock::now();
  return true;
}

void SensorsSyncNode::reset_sync_state()
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  left_buffer_.clear();
  right_buffer_.clear();
  unmatched_frame_times_.clear();
  last_synced_frame_id_ = 0;
  last_left_frame_id_ = 0;
  last_right_frame_id_ = 0;
  camera_time_initialized_ = false;
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
  // Frames carry the camera's own clock, not ROS time, so age is measured against the newest
  // frame in the same buffer. This bounds the buffer of a camera whose counterpart has stopped
  // delivering (~1.8 MB per frame).
  const auto max_age_ns = static_cast<uint64_t>(camera_buffer_duration_ * 1e9);
  auto trim_old = [&](std::deque<CameraFrame> & buffer) {
    while (!buffer.empty() &&
           buffer.back().internal_timestamp_ns > buffer.front().internal_timestamp_ns + max_age_ns) {
      buffer.pop_front();
    }
  };

  trim_old(left_buffer_);
  trim_old(right_buffer_);
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
    // Dropped a frame the other camera never delivered
    unmatched_frame_times_.push_back(std::chrono::steady_clock::now());
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
  trim_old_data();
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

  // Anchor timestamp sync on the first stereo pair. Frame ID #0 never reaches the buffers, so
  // this is frame #1 unless that frame was lost on either camera.
  if (!camera_time_initialized_) {
    camera_start_ts_left_ = left->internal_timestamp_ns;
    camera_start_ts_right_ = right->internal_timestamp_ns;
    camera_start_ros_time_ = this->now();
    camera_time_initialized_ = true;

    RCLCPP_INFO(this->get_logger(),
      "Camera start time anchored from first stereo pair (frame #%lu): %.9f",
      left->frame_id, camera_start_ros_time_.seconds());
  }

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
  last_pair_time_ = std::chrono::steady_clock::now();

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
    node->shutdown();
    rclcpp::shutdown();
    return 1;
  }
  // A SIGINT during the camera open has already shut ROS down, and spinning would throw
  if (rclcpp::ok()) {
    rclcpp::spin(node);
  }
  node->shutdown();
  rclcpp::shutdown();
  return node->has_fatal_error() ? 1 : 0;
}
