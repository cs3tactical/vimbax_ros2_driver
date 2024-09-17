#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/header.hpp"

class MultiTopicSubscriber : public rclcpp::Node
{
public:
    MultiTopicSubscriber():
        Node("multi_topic_subscriber"),
        diff_(0),
        stamp1_(0),
        stamp2_(0)
    {
        // Subscriber to "topic1"
        initialize_subscription1();

        // Subscriber to "topic2"
        initialize_subscription2();

        // Calculate the difference between the two timestamps
        diff_ = 0;
        stamp1_ = 0;
        stamp2_ = 0;
    }

private:
    void initialize_subscription1() 
    {
        subscription1_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/vimbax_camera_ir/camera_info", 10, std::bind(&MultiTopicSubscriber::topic1_callback, this, std::placeholders::_1));
    }

    void initialize_subscription2() 
    {
        subscription2_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/vimbax_camera_rgb/camera_info", 10, std::bind(&MultiTopicSubscriber::topic2_callback, this, std::placeholders::_1));
    }

    // Callback function for "topic1"
    void topic1_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) 
    {
        // RCLCPP_INFO(this->get_logger(), "Received on topic1: '%d'", msg->header.stamp.nanosec);
        stamp1_ = static_cast<double>(msg->header.stamp.nanosec);
        // diff_ = stamp1_ - stamp2_;
        // RCLCPP_INFO(this->get_logger(), "Difference1: '%f'", diff_);
    }

    // Callback function for "topic2"
    void topic2_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) 
    {
        // RCLCPP_INFO(this->get_logger(), "Received on topic2: '%d'", msg->header.stamp.nanosec);
        stamp2_ = static_cast<double>(msg->header.stamp.nanosec);
        diff_ = (stamp2_ - stamp1_)/1000000.0;
        RCLCPP_INFO(this->get_logger(), "Time difference in ms: '%f'", diff_);

        if (diff_ > 2.0) {
            RCLCPP_INFO(this->get_logger(), "Reset subscription to topic2");
            subscription2_.reset();

            int diff = static_cast<int>(diff_);
            rclcpp::sleep_for(std::chrono::milliseconds(diff));

            initialize_subscription2();
            }        
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription1_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription2_;

    // Variables
    double diff_;
    int stamp1_;
    int stamp2_;
};

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create an instance of the subscriber node
    auto node = std::make_shared<MultiTopicSubscriber>();

    // Spin the node (handle callbacks)
    rclcpp::spin(node);

    // Shutdown the ROS 2 system
    rclcpp::shutdown();
    return 0;
}
