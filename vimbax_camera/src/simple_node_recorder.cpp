#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
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
        : Node("stereo_recorder", options)
    {
        // Declare and retrieve parameters
        this->declare_parameter<std::string>("topic_left", "/vimbax_camera_left/image_raw");
        this->declare_parameter<std::string>("recording_name", "stereo_recording");
        this->declare_parameter<std::string>("recording_path", "/home/peterpan/stereo_bag");

        this->get_parameter("topic_left", topic_left_);
        this->get_parameter("recording_name", recording_name_);
        this->get_parameter("recording_path", recording_path_);

        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

        // Subscriber to left image topic
        left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_left_, qos_profile, std::bind(&StereoRecorder::leftImageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "StereoRecorder node initialized.");

        // Automatically start recording
        initializeRecorder();
    }

    ~StereoRecorder()
    {
        if (writer_)
        {
            writer_.reset();
            RCLCPP_INFO(this->get_logger(), "Recording stopped.");
        }
    }

private:
    void initializeRecorder()
    {
        // Generate timestamped bag file path
        std::time_t t = std::time(0);
        std::tm* now = std::localtime(&t);
        std::string timestamp = std::to_string(now->tm_year + 1900) + "_" +
                                 std::to_string(now->tm_mon + 1) + "_" +
                                 std::to_string(now->tm_mday) + "-" +
                                 std::to_string(now->tm_hour) + "_" +
                                 std::to_string(now->tm_min) + "_" +
                                 std::to_string(now->tm_sec);

        std::filesystem::path bag_file_path = std::filesystem::path(recording_path_) / (recording_name_ + "_" + timestamp);

        // Initialize the rosbag writer
        writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_file_path.string();
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        writer_->open(storage_options, converter_options);

        // Create topic metadata
        rosbag2_storage::TopicMetadata left_image_metadata = {
            topic_left_, "sensor_msgs/msg/Image", "cdr", ""}; // "" for default QoS profiles

        writer_->create_topic(left_image_metadata);

        RCLCPP_INFO(this->get_logger(), "Recording started at %s", bag_file_path.string().c_str());
    }

    void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (writer_)
        {
            auto serialized_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            serialized_msg->topic_name = topic_left_;
            serialized_msg->time_stamp = msg->header.stamp.nanosec + static_cast<uint64_t>(msg->header.stamp.sec) * 1e9;

            rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
            rclcpp::SerializedMessage serialized_data;
            serializer.serialize_message(msg.get(), &serialized_data);

            serialized_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();
            *serialized_msg->serialized_data = serialized_data.get_rcl_serialized_message();

            writer_->write(serialized_msg);
        }
    }

    // Parameters
    std::string topic_left_;
    std::string recording_name_;
    std::string recording_path_;

    // Rosbag writer
    std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
};

} // namespace stereo_recorder

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_recorder::StereoRecorder)

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create the node with default options
    auto node = std::make_shared<stereo_recorder::StereoRecorder>(rclcpp::NodeOptions());

    // Spin the node
    rclcpp::spin(node);

    // Shutdown the ROS2 system after the node stops
    rclcpp::shutdown();
    return 0;
}
