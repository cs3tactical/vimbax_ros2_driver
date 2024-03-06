// Copyright 2024 Allied Vision Technologies GmbH. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VIMBAX_CAMERA__VIMBAX_CAMERA_HELPER_HPP_
#define VIMBAX_CAMERA__VIMBAX_CAMERA_HELPER_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>


struct feature_float_info
{
  _Float64 min;
  _Float64 max;
  _Float64 inc;
  bool inc_available;
};

struct feature_flags
{
  bool flag_none;
  bool flag_read;
  bool flag_write;
  bool flag_volatile;
  bool flag_modify_write;
};
struct feature_info
{
  std::string name;
  std::string category;
  std::string display_name;
  std::string sfnc_namespace;
  std::string unit;
  uint32_t data_type;           // Data type corresponding to VmbFeatureDataType
  feature_flags flags;
  uint32_t polling_time;
};

namespace vimbax_camera
{
struct SFNCFeatures
{
  static constexpr std::string_view PixelFormat = "PixelFormat";
  static constexpr std::string_view AcquisitionStart = "AcquisitionStart";
  static constexpr std::string_view AcquisitionStop = "AcquisitionStop";
  static constexpr std::string_view Width = "Width";
  static constexpr std::string_view Height = "Height";
  static constexpr std::string_view TriggerMode = "TriggerMode";
  static constexpr std::string_view TriggerSource = "TriggerSource";
  static constexpr std::string_view DeviceFirmwareVersion = "DeviceFirmwareVersion";
  static constexpr std::string_view DeviceUserId = "DeviceUserID";
  static constexpr std::string_view AcquisitionFrameRate = "AcquisitionFrameRate";

  static constexpr std::string_view InterfaceId = "InterfaceID";
  static constexpr std::string_view TransportLayerId = "TLID";

  static constexpr std::string_view GevDeviceIPAddress = "GevDeviceIPAddress";
  static constexpr std::string_view GevDeviceMACAddress = "GevDeviceMACAddress";

  static constexpr std::string_view StreamBufferAlignment = "StreamBufferAlignment";

  static constexpr std::string_view EventSelector = "EventSelector";
  static constexpr std::string_view EventNotification = "EventNotification";
  static constexpr std::string_view EventCameraDiscovery = "EventCameraDiscovery";
  static constexpr std::string_view EventCameraDiscoveryCameraID = "EventCameraDiscoveryCameraID";
  static constexpr std::string_view EventCameraDiscoveryType = "EventCameraDiscoveryType";
};
}  // namespace vimbax_camera

namespace vimbax_camera::helper
{

std::string_view vmb_error_to_string(int32_t error_code);

rclcpp::Logger get_logger();

rclcpp::Node::SharedPtr create_node(const std::string & name, const rclcpp::NodeOptions & options);

void left_shift16(void * out, const void * in, size_t size, int shift);
}  // namespace vimbax_camera::helper

#endif  // VIMBAX_CAMERA__VIMBAX_CAMERA_HELPER_HPP_