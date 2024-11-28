#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <string>
#include <cstdlib> // for std::system

class StereoSyncNode : public rclcpp::Node {
public:
    StereoSyncNode() : Node("stereo_sync_node") {
        // Declare and get parameters
        left_camera_topic_ = this->declare_parameter<std::string>("left_camera_topic", "/vimbax_camera_left/image_raw");
        right_camera_topic_ = this->declare_parameter<std::string>("right_camera_topic", "/vimbax_camera_right/image_raw");
        left_camera_sync_topic_ = this->declare_parameter<std::string>("left_camera_sync_topic", "/vimbax_camera_left/image_raw_sync");
        right_camera_sync_topic_ = this->declare_parameter<std::string>("right_camera_sync_topic", "/vimbax_camera_right/image_raw_sync");

        // Initialize variables
        trigger_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

        // Subscribers
        left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            left_camera_topic_, 10, std::bind(&StereoSyncNode::left_callback, this, std::placeholders::_1));
        right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            right_camera_topic_, 10, std::bind(&StereoSyncNode::right_callback, this, std::placeholders::_1));

        // Publishers
        left_pub_ = this->create_publisher<sensor_msgs::msg::Image>(left_camera_sync_topic_, 10);
        right_pub_ = this->create_publisher<sensor_msgs::msg::Image>(right_camera_sync_topic_, 10);

        // Trigger cameras
        trigger_cameras();

        RCLCPP_INFO(this->get_logger(), "Stereo synchronization node initialized and cameras triggered.");
    }

    ~StereoSyncNode() {
        // Cleanup PWM signal
        RCLCPP_INFO(this->get_logger(), "Disabling PWM signal...");
        std::system("echo 0 > /sys/class/pwm/pwmchip0/pwm0/enable");
        RCLCPP_INFO(this->get_logger(), "PWM signal disabled.");
    }

private:
    std::string left_camera_topic_;
    std::string right_camera_topic_;
    std::string left_camera_sync_topic_;
    std::string right_camera_sync_topic_;

    rclcpp::Time trigger_time_;
    double left_start_time_ = 0.0;
    double right_start_time_ = 0.0;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;

    void trigger_cameras() {
        // Execute the PWM commands to trigger cameras
        RCLCPP_INFO(this->get_logger(), "Starting PWM signal to trigger cameras...");

        std::system("echo 0 > /sys/class/pwm/pwmchip0/export");
        std::system("echo 66666666 > /sys/class/pwm/pwmchip0/pwm0/period");
        std::system("echo 33333333 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle");
        std::system("echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable");

        trigger_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Trigger signal sent at ROS time: %lf.%09ld", 
                    trigger_time_.seconds(), trigger_time_.nanoseconds());
    }

    void left_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (left_start_time_ == 0.0) {
            left_start_time_ = timestamp_to_double(msg->header.stamp);
            RCLCPP_INFO(this->get_logger(), "Left first frame");          
        }
        sync_and_publish(msg, left_start_time_, left_pub_);
    }

    void right_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (right_start_time_ == 0.0) {
            right_start_time_ = timestamp_to_double(msg->header.stamp);
            RCLCPP_INFO(this->get_logger(), "Right first frame");  
        }
        sync_and_publish(msg, right_start_time_, right_pub_);
    }

    void sync_and_publish(const sensor_msgs::msg::Image::SharedPtr msg, double start_time,
                          const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher) {
        double current_camera_time = timestamp_to_double(msg->header.stamp);
        double time_offset = current_camera_time - start_time;

        rclcpp::Time corrected_time = trigger_time_ + rclcpp::Duration::from_seconds(time_offset);
        msg->header.stamp = corrected_time;

        publisher->publish(*msg);
    }

    double timestamp_to_double(const builtin_interfaces::msg::Time &timestamp) {
        return timestamp.sec + timestamp.nanosec * 1e-9;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoSyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
