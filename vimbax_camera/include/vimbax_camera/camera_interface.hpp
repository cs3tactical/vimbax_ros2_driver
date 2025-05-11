#ifndef VIMBAX_CAMERA_SYNC__CAMERA_INTERFACE_HPP_
#define VIMBAX_CAMERA_SYNC__CAMERA_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <functional>
#include <string>

#include <vimbax_camera/loader/vmbc_api.hpp>
#include <vimbax_camera/vimbax_camera.hpp>

namespace vimbax_camera_sync
{

struct CameraMetadata
{
  double exposure_time_us = 0.0;

  double device_temp_c = 0.0;
  std::string temp_status;
  
  double gain_db = 0.0;
};

struct CameraFrame
{
  sensor_msgs::msg::Image image;
  sensor_msgs::msg::CameraInfo info;
  uint64_t frame_id = 0;                 // From Vimba: frame->get_frame_id()
  uint64_t internal_timestamp_ns = 0;   // From Vimba: frame->get_timestamp_ns()
  CameraMetadata metadata;
};

class CameraInterface
{
public:
  using FrameCallback = std::function<void(const CameraFrame &)>;

  CameraInterface(rclcpp::Node * node, const std::string & camera_id);
  ~CameraInterface();

  bool initialize(FrameCallback callback);
  bool is_ready() const;

private:
  rclcpp::Node * node_;
  std::string camera_id_;
  bool ready_ = false;
  uint64_t last_frame_id_ = 0;
  FrameCallback frame_callback_;

  std::shared_ptr<vimbax_camera::VmbCAPI> api_;
  std::shared_ptr<vimbax_camera::VimbaXCamera> camera_;

  void populate_metadata(CameraMetadata & meta);
};

}  // namespace vimbax_camera_sync

#endif  // VIMBAX_CAMERA_SYNC__CAMERA_INTERFACE_HPP_
