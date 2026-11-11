// camera_interface.cpp
#include "vimbax_camera/camera_interface.hpp"

#include <vimbax_camera/vimbax_camera_helper.hpp>
#include <mutex>
#include <thread>

namespace vimbax_camera_sync
{

// Struct to hold encoding information
struct EncodingInfo {
  std::string encoding;
  int num_channels;
};

// Helper function to map pixel format string to ROS encoding and channel count
EncodingInfo get_encoding_from_pixel_format(const std::string & pixel_format)
{
  if (pixel_format == "Mono8") {
    return {"mono8", 1};
  } else if (pixel_format == "Mono12") {
    return {"mono16", 1};  // ROS has no native mono12 encoding
  } else if (pixel_format == "RGB8" || pixel_format == "BGR8") {
    return {"rgb8", 3};  // You can switch to "bgr8" if needed
  } else if (pixel_format == "YUV422") {
    return {"yuv422", 2};  // Approximate
  } else {
    // Fallback: assume 3-channel RGB
    return {"rgb8", 3};
  }
}

// Constructor
CameraInterface::CameraInterface(
  rclcpp::Node * node,
  const std::string & camera_id)
: node_(node), camera_id_(camera_id)
{
}

// Destructor
CameraInterface::~CameraInterface()
{
  if (camera_ && camera_->is_streaming()) {
    camera_->stop_streaming();
  }
}

// Main initialization method
bool CameraInterface::initialize(FrameCallback callback)
{
  frame_callback_ = callback;

  RCLCPP_INFO(node_->get_logger(), "[CameraInterface] Initializing VimbaX API...");
  api_ = vimbax_camera::VmbCAPI::get_instance();
  if (!api_) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load VimbaX API");
    return false;
  }

  // One open at a time: open() lists the cameras, and the VmbCameraInfo_t strings it compares
  // point into a list that any other VmbCamerasList call rebuilds. Opening both cameras in
  // parallel read freed strings (segfault) or saw the list change size (VmbErrorMoreData).
  static std::mutex open_mutex;
  {
    std::lock_guard<std::mutex> lock(open_mutex);
    camera_ = vimbax_camera::VimbaXCamera::open(api_, camera_id_);
  }
  if (!camera_) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open camera: %s", camera_id_.c_str());
    return false;
  }

  auto info = camera_->camera_info_get();
  if (!info) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to query camera info");
    return false;
  }

  std::string pixel_format = info->pixel_format;
  EncodingInfo enc_info = get_encoding_from_pixel_format(pixel_format);

  on_frame_ = [this, enc_info](std::shared_ptr<vimbax_camera::VimbaXCamera::Frame> frame) {
      CameraFrame camera_frame;
      camera_frame.frame_id = frame->get_frame_id();
      camera_frame.internal_timestamp_ns = frame->get_timestamp_ns();

      if (last_frame_id_) {
        auto diff = camera_frame.frame_id - last_frame_id_;
        if (diff > 1) {
          RCLCPP_WARN(node_->get_logger(),
            "[CameraInterface][%s] %lu frame(s) dropped! (last: %lu, current: %lu)",
            camera_id_.c_str(), diff - 1, last_frame_id_, camera_frame.frame_id);
        }
      }
      last_frame_id_ = camera_frame.frame_id;

      sensor_msgs::msg::Image & image = camera_frame.image;
      sensor_msgs::msg::CameraInfo & info = camera_frame.info;

      // ---- Header ----
      image.header.frame_id = camera_id_;

      std::chrono::nanoseconds ts_ns(frame->get_timestamp_ns());
      auto seconds = std::chrono::duration_cast<std::chrono::seconds>(ts_ns);
      auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(ts_ns - seconds);
      image.header.stamp.sec = static_cast<int32_t>(seconds.count());
      image.header.stamp.nanosec = static_cast<uint32_t>(nanos.count());

      // ---- Image data ----
      image.height = frame->height;
      image.width = frame->width;
      image.encoding = enc_info.encoding;
      image.step = frame->width * enc_info.num_channels;
      image.data = frame->data;

      // ---- Camera Info ----
      info.header = image.header;
      info.width = image.width;
      info.height = image.height;

      // ----- metadata -------
      populate_metadata(camera_frame.metadata);

      // ---- Callback ----
      if (frame_callback_) {
        try {
          frame_callback_(camera_frame);
        } catch (const rclcpp::exceptions::RCLError &) {
          // Frames keep arriving after SIGINT has shut ROS down, until the node closes the
          // cameras, and publishing them throws. Letting that escape this thread would abort
          // the process with the stream still open.
          if (rclcpp::ok()) {
            throw;
          }
        }
      }

      // ---- Requeue frame ----
      auto err = frame->queue();
      if (err != VmbErrorSuccess) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to re-queue frame (%d)", err);
      }
    };

  if (!start_streaming()) {
    return false;
  }

  ready_ = true;
  return true;
}

// Returns true if streaming and initialized
bool CameraInterface::is_ready() const
{
  return ready_;
}

bool CameraInterface::stop_streaming()
{
  auto result = camera_->stop_streaming();
  if (!result) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to stop streaming on camera: %s", camera_id_.c_str());
    return false;
  }
  return true;
}

bool CameraInterface::start_streaming()
{
  // The frame ID may restart from 0 on a new stream
  last_frame_id_ = 0;

  int buffer_count = 10;
  auto result = camera_->start_streaming(buffer_count, on_frame_);
  if (!result) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to start streaming on camera: %s", camera_id_.c_str());
    return false;
  }
  return true;
}

void CameraInterface::populate_metadata(CameraMetadata & meta)
{
  // Exposure settings
  auto exposure_time = camera_->feature_float_get("ExposureTime");
  meta.exposure_time_us = exposure_time ? *exposure_time : 0.0;

  // Device temperature
  auto device_temp = camera_->feature_float_get("DeviceTemperature");
  meta.device_temp_c = device_temp ? *device_temp : 0.0;

  auto temp_status = camera_->feature_enum_get("DeviceTemperatureStatus");
  meta.temp_status = temp_status ? *temp_status : "UNKNOWN";

  // Gain settings
  auto gain = camera_->feature_float_get("Gain");
  meta.gain_db = gain ? *gain : 0.0;
}


}  // namespace vimbax_camera_sync
