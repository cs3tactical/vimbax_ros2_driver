#include "vimbax_camera/camera_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

using namespace vimbax_camera_sync;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("camera_interface_test");

  auto pub = image_transport::create_camera_publisher(node.get(), "image_raw");

  std::string camera_id = "DEV_00012C050ADD";  // use actual ID if needed
  CameraInterface camera(node.get(), camera_id);

  bool ok = camera.initialize([&](const CameraFrame & frame) {
    static int count = 0;
    RCLCPP_INFO(node->get_logger(),
      "[Frame %d] ID: %lu, Timestamp: %.3f s, Size: %dx%d",
      count++,
      frame.frame_id,
      frame.internal_timestamp_ns / 1e9,
      frame.image.width,
      frame.image.height);

    pub.publish(frame.image, frame.info);
  });

  if (!ok) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize camera.");
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
