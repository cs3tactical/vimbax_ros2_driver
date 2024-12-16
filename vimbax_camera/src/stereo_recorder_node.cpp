#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp/serialization.hpp>
#include <filesystem>
#include <string>
#include <memory>
#include <ctime>

namespace stereo_recorder
{
class StereoRecorder : public rclcpp::Node
{
public:
    StereoRecorder(const rclcpp::NodeOptions &options)
        : Node("stereo_recorder", options), is_recording_(false)
    {
        this->declare_parameter<std::string>("topic_left", "/camera/left/image_raw");
        this->declare_parameter<std::string>("topic_right", "/camera/right/image_raw");
        this->declare_parameter<std::string>("topic_camera_info_left", "/camera/left/camera_info");
        this->declare_parameter<std::string>("topic_camera_info_right", "/camera/right/camera_info");
        this->declare_parameter<std::string>("recording_name", "stereo_recording");
        this->declare_parameter<std::string>("recording_path", "/tmp");
        this->declare_parameter<std::string>("recording_format", "mcap");

        this->get_parameter("topic_left", topic_left_);
        this->get_parameter("topic_right", topic_right_);
        this->get_parameter("topic_camera_info_left", topic_camera_info_left_);
        this->get_parameter("topic_camera_info_right", topic_camera_info_right_);
        this->get_parameter("recording_name", recording_name_);
        this->get_parameter("recording_path", recording_path_);
        this->get_parameter("recording_format", recording_format_);

        left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_left_, 10, std::bind(&StereoRecorder::leftImageCallback, this, std::placeholders::_1));
        right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_right_, 10, std::bind(&StereoRecorder::rightImageCallback, this, std::placeholders::_1));
        left_camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            topic_camera_info_left_, 10, std::bind(&StereoRecorder::leftCameraInfoCallback, this, std::placeholders::_1));
        right_camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            topic_camera_info_right_, 10, std::bind(&StereoRecorder::rightCameraInfoCallback, this, std::placeholders::_1));

        start_record_service_ = this->create_service<std_srvs::srv::Trigger>(
            "start_record", std::bind(&StereoRecorder::startRecord, this, std::placeholders::_1, std::placeholders::_2));
        stop_record_service_ = this->create_service<std_srvs::srv::Trigger>(
            "stop_record", std::bind(&StereoRecorder::stopRecord, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "StereoRecorder node initialized.");
    }

    ~StereoRecorder()
    {
        if (is_recording_)
        {
            stopRecordingInternal();
        }
    }

private:
    void initializeRecorder()
    {
        std::time_t t = std::time(0);
        std::tm* now = std::localtime(&t);
        std::string timestamp = std::to_string(now->tm_year + 1900) + "_" +
                                 std::to_string(now->tm_mon + 1) + "_" +
                                 std::to_string(now->tm_mday) + "-" +
                                 std::to_string(now->tm_hour) + "_" +
                                 std::to_string(now->tm_min) + "_" +
                                 std::to_string(now->tm_sec);

        std::filesystem::path bag_file_path = std::filesystem::path(recording_path_) / (recording_name_ + "_" + timestamp);

        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_file_path.string();
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = recording_format_;

        writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
        writer_->open(storage_options, converter_options);

        is_recording_ = true;

        RCLCPP_INFO(this->get_logger(), "Recording started at %s", bag_file_path.string().c_str());
    }

    void stopRecordingInternal()
    {
        if (writer_)
        {
            writer_.reset();
            is_recording_ = false;
            RCLCPP_INFO(this->get_logger(), "Recording stopped.");
        }
    }

    void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (is_recording_ && writer_)
        {
            auto serialized_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            serialized_msg->topic_name = topic_left_;
            serialized_msg->time_stamp = msg->header.stamp.nanosec + static_cast<uint64_t>(msg->header.stamp.sec) * 1e9;

            rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
            rclcpp::SerializedMessage serialized_data;
            serializer.serialize_message(msg.get(), &serialized_data);

            serialized_msg->serialized_data = std::make_shared<std::vector<uint8_t>>(
                serialized_data.get_rcl_serialized_message().buffer,
                serialized_data.get_rcl_serialized_message().buffer + serialized_data.size());

            writer_->write(serialized_msg);
        }
    }

    void rightImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (is_recording_ && writer_)
        {
            auto serialized_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            serialized_msg->topic_name = topic_right_;
            serialized_msg->time_stamp = msg->header.stamp.nanosec + static_cast<uint64_t>(msg->header.stamp.sec) * 1e9;

            rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
            rclcpp::SerializedMessage serialized_data;
            serializer.serialize_message(msg.get(), &serialized_data);

            serialized_msg->serialized_data = std::make_shared<std::vector<uint8_t>>(
                serialized_data.get_rcl_serialized_message().buffer,
                serialized_data.get_rcl_serialized_message().buffer + serialized_data.size());

            writer_->write(serialized_msg);
        }
    }

    void leftCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (is_recording_ && writer_)
        {
            auto serialized_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            serialized_msg->topic_name = topic_camera_info_left_;
            serialized_msg->time_stamp = msg->header.stamp.nanosec + static_cast<uint64_t>(msg->header.stamp.sec) * 1e9;

            rclcpp::Serialization<sensor_msgs::msg::CameraInfo> serializer;
            rclcpp::SerializedMessage serialized_data;
            serializer.serialize_message(msg.get(), &serialized_data);

            serialized_msg->serialized_data = std::make_shared<std::vector<uint8_t>>(
                serialized_data.get_rcl_serialized_message().buffer,
                serialized_data.get_rcl_serialized_message().buffer + serialized_data.size());

            writer_->write(serialized_msg);
        }
    }

    void rightCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (is_recording_ && writer_)
        {
            auto serialized_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            serialized_msg->topic_name = topic_camera_info_right_;
            serialized_msg->time_stamp = msg->header.stamp.nanosec + static_cast<uint64_t>(msg->header.stamp.sec) * 1e9;

            rclcpp::Serialization<sensor_msgs::msg::CameraInfo> serializer;
            rclcpp::SerializedMessage serialized_data;
            serializer.serialize_message(msg.get(), &serialized_data);

            serialized_msg->serialized_data = std::make_shared<std::vector<uint8_t>>(
                serialized_data.get_rcl_serialized_message().buffer,
                serialized_data.get_rcl_serialized_message().buffer + serialized_data.size());

            writer_->write(serialized_msg);
        }
    }

    void startRecord(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (is_recording_)
        {
            response->success = false;
            response->message = "Recording is already in progress.";
        }
        else
        {
            initializeRecorder();
            response->success = true;
            response->message = "Recording started successfully.";
        }
    }

    void stopRecord(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!is_recording_)
        {
            response->success = false;
            response->message = "No active recording to stop.";
        }
        else
        {
            stopRecordingInternal();
            response->success = true;
            response->message = "Recording stopped successfully.";
        }
    }

    // Parameters
    std::string topic_left_;
    std::string topic_right_;
    std::string topic_camera_info_left_;
    std::string topic_camera_info_right_;
    std::string recording_name_;
    std::string recording_path_;
    std::string recording_format_;

    // State
    bool is_recording_;
    std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_camera_info_sub_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_record_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_record_service_;
};

} // namespace stereo_recorder

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_recorder::StereoRecorder)
