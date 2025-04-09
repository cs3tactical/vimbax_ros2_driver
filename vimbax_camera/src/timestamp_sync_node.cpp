#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vimbax_camera_msgs/msg/stereo_sync_status.hpp"

#include <string>
#include <sstream>
#include <map>
#include <deque>
#include <cmath>

class TimestampSyncChecker : public rclcpp::Node
{
public:
  TimestampSyncChecker()
  : Node("timestamp_sync_checker")
  {
    // Declare and get the moving average size parameter
    this->declare_parameter<int>("moving_average_window", 50);
    moving_average_size_ = this->get_parameter("moving_average_window").as_int();

    RCLCPP_INFO(this->get_logger(), "Using moving average window size: %zu", moving_average_size_);

    left_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/vimbax_camera_left/timestamp_offset", 10,
      std::bind(&TimestampSyncChecker::left_callback, this, std::placeholders::_1));

    right_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/vimbax_camera_right/timestamp_offset", 10,
      std::bind(&TimestampSyncChecker::right_callback, this, std::placeholders::_1));

    sync_status_pub_ = this->create_publisher<vimbax_camera_msgs::msg::StereoSyncStatus>(
      "/stereo_time_sync", 10);
  }

private:
  struct TimestampData
  {
    double ros_time;
    double camera_time;
    double time_from_start;
  };

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr left_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr right_sub_;
  rclcpp::Publisher<vimbax_camera_msgs::msg::StereoSyncStatus>::SharedPtr sync_status_pub_;

  std::map<int, TimestampData> left_buffer_;
  std::map<int, TimestampData> right_buffer_;
  std::deque<double> offset_history_ros_;
  std::deque<double> offset_history_start_;

  const size_t max_buffer_size_ = 100;
  size_t moving_average_size_;  // <-- now configurable!
  bool moving_average_ready_ = false;

  void left_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    auto [frame_id, data] = parse_msg(msg->data);
    left_buffer_[frame_id] = data;
    match_frames(frame_id);
  }

  void right_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    auto [frame_id, data] = parse_msg(msg->data);
    right_buffer_[frame_id] = data;
    match_frames(frame_id);
  }

  std::pair<int, TimestampData> parse_msg(const std::string & input)
  {
    std::stringstream ss(input);
    std::string token;
    int frame_id;
    double ros_time, camera_time, time_from_start;
    std::getline(ss, token, ',');
    frame_id = std::stoi(token);
    std::getline(ss, token, ',');
    ros_time = std::stod(token);
    std::getline(ss, token, ',');
    camera_time = std::stod(token);
    std::getline(ss, token, ',');
    time_from_start = std::stod(token);

    TimestampData data{ros_time, camera_time, time_from_start};
    return {frame_id, data};
  }

  void match_frames(int frame_id)
  {
    if (left_buffer_.count(frame_id) && right_buffer_.count(frame_id))
    {
      auto left = left_buffer_[frame_id];
      auto right = right_buffer_[frame_id];

      double ros_time_offset = right.ros_time - left.ros_time;
      double start_time_offset = right.time_from_start - left.time_from_start;

      offset_history_ros_.push_back(ros_time_offset);
      offset_history_start_.push_back(start_time_offset);

      if (offset_history_ros_.size() > moving_average_size_)
        offset_history_ros_.pop_front();
      if (offset_history_start_.size() > moving_average_size_)
        offset_history_start_.pop_front();

      // Publish the sync status
      auto msg = vimbax_camera_msgs::msg::StereoSyncStatus();
    //   msg.header.stamp = this->now();
    //   msg.header.frame_id = "stereo_sync";

      msg.frame_id = frame_id;
      msg.ros_left = left.ros_time;
      msg.ros_right = right.ros_time;
      msg.ros_offset = ros_time_offset;
      msg.start_offset = start_time_offset;

      msg.left_start_time = left.time_from_start;
      msg.right_start_time = right.time_from_start;

      if (offset_history_ros_.size() == moving_average_size_)
      {
        msg.moving_avg_ros_offset = average(offset_history_ros_);
        msg.moving_avg_start_offset = average(offset_history_start_);

        if (!moving_average_ready_)
        {
          RCLCPP_INFO(this->get_logger(), "Moving average ready after %zu frames.", moving_average_size_);
          moving_average_ready_ = true;
        }
      }
      else
      {
        msg.moving_avg_ros_offset = 0.0;
        msg.moving_avg_start_offset = 0.0;
      }

      sync_status_pub_->publish(msg);

      // Warn if offsets are too high
    //   if (std::abs(ros_time_offset) > 0.005)
    //     RCLCPP_WARN(this->get_logger(), "High ROS time offset: %.6f s", ros_time_offset);
    //   if (std::abs(start_time_offset) > 0.005)
    //     RCLCPP_WARN(this->get_logger(), "High start time offset: %.6f s", start_time_offset);

      left_buffer_.erase(frame_id);
      right_buffer_.erase(frame_id);
    }

    // Clean old unmatched frames if buffer gets too big
    if (left_buffer_.size() > max_buffer_size_)
    {
      RCLCPP_WARN(this->get_logger(), "Dropping oldest left frame to limit buffer size");
      left_buffer_.erase(left_buffer_.begin());
    }
    if (right_buffer_.size() > max_buffer_size_)
    {
      RCLCPP_WARN(this->get_logger(), "Dropping oldest right frame to limit buffer size");
      right_buffer_.erase(right_buffer_.begin());
    }
  }

  double average(const std::deque<double> & data)
  {
    double sum = 0.0;
    for (auto d : data)
      sum += d;
    return sum / static_cast<double>(data.size());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimestampSyncChecker>());
  rclcpp::shutdown();
  return 0;
}
