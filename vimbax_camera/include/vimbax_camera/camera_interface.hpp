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

struct CameraFrame
{
  sensor_msgs::msg::Image image;
  sensor_msgs::msg::CameraInfo info;
  uint64_t frame_id = 0;                 // From Vimba: frame->get_frame_id()
  uint64_t internal_timestamp_ns = 0;   // From Vimba: frame->get_timestamp_ns()
};

class CameraInterface
{
public:
  using FrameCallback = std::function<void(const CameraFrame &)>;

  CameraInterface(const rclcpp::Node::SharedPtr & node, const std::string & camera_id);
  ~CameraInterface();

  bool initialize(FrameCallback callback);
  bool is_ready() const;

private:
  rclcpp::Node::SharedPtr node_;
  std::string camera_id_;
  bool ready_ = false;
  FrameCallback frame_callback_;

  std::shared_ptr<vimbax_camera::VmbCAPI> api_;
  std::shared_ptr<vimbax_camera::VimbaXCamera> camera_;
};

}  // namespace vimbax_camera_sync

#endif  // VIMBAX_CAMERA_SYNC__CAMERA_INTERFACE_HPP_
