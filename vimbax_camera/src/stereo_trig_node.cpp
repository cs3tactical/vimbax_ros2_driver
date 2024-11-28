#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <string>
#include <cstdlib> // for std::system
#include <memory>  // for std::shared_ptr
#include "vimbax_camera_msgs/srv/trigger_time.hpp" // Include your custom service definition

using namespace std::chrono_literals;

class StereoTrigNode : public rclcpp::Node {
public:
    StereoTrigNode() : Node("stereo_trig_node") {
        // Initialize parameters
        trigger_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

        // Service clients
        left_camera_client_ = this->create_client<vimbax_camera_msgs::srv::TriggerTime>("/vimbax_camera_left/set_trigger_time");
        right_camera_client_ = this->create_client<vimbax_camera_msgs::srv::TriggerTime>("/vimbax_camera_right/set_trigger_time");

        // Initialize synchronization process
        send_trigger_time(1s);
    }

    ~StereoTrigNode() {
        // Cleanup PWM signal
        RCLCPP_INFO(this->get_logger(), "Disabling PWM signal...");
        std::system("echo 0 > /sys/class/pwm/pwmchip0/pwm0/enable");
        RCLCPP_INFO(this->get_logger(), "PWM signal disabled.");
    }

private:
    rclcpp::Time trigger_time_;
    rclcpp::Client<vimbax_camera_msgs::srv::TriggerTime>::SharedPtr left_camera_client_;
    rclcpp::Client<vimbax_camera_msgs::srv::TriggerTime>::SharedPtr right_camera_client_;

    void send_trigger_time(std::chrono::seconds delay) {
        RCLCPP_INFO(this->get_logger(), "Waiting for services to be available...");
        left_camera_client_->wait_for_service();
        right_camera_client_->wait_for_service();

        RCLCPP_INFO(this->get_logger(), "Services available in both cameras");

        // Set the trigger time to current time + delay
        trigger_time_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(delay.count());

        auto request = std::make_shared<vimbax_camera_msgs::srv::TriggerTime::Request>();
        request->trigger_time = trigger_time_;

        // Send requests to both camera nodes and use shared_ptr to handle futures
        auto left_future = std::make_shared<rclcpp::Client<vimbax_camera_msgs::srv::TriggerTime>::SharedFuture>(
            left_camera_client_->async_send_request(request));
        auto right_future = std::make_shared<rclcpp::Client<vimbax_camera_msgs::srv::TriggerTime>::SharedFuture>(
            right_camera_client_->async_send_request(request));

        RCLCPP_INFO(this->get_logger(), "Trigger time sent to both cameras");

        // Create a timer to trigger the cameras after the delay
        auto timer = this->create_wall_timer(
            delay, [this, left_future, right_future, delay]() {
                bool left_ack = false, right_ack = false;

                if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), *left_future, 500ms) == rclcpp::FutureReturnCode::SUCCESS) {
                    left_ack = left_future->get()->success;
                }

                if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), *right_future, 500ms) == rclcpp::FutureReturnCode::SUCCESS) {
                    right_ack = right_future->get()->success;
                }

                // Check for acknowledgments
                if (left_ack && right_ack) {
                    RCLCPP_INFO(this->get_logger(), "Both cameras acknowledged the trigger time.");
                    // Trigger the cameras
                    trigger_cameras();
                } else {
                    // Log which camera did not acknowledge and retry with a larger delay
                    if (!left_ack) {
                        RCLCPP_WARN(this->get_logger(), "Left camera did not acknowledge trigger time.");
                    }
                    if (!right_ack) {
                        RCLCPP_WARN(this->get_logger(), "Right camera did not acknowledge trigger time.");
                    }

                    RCLCPP_WARN(this->get_logger(), "Retrying with a delay of 2 seconds...");
                    send_trigger_time(2s);
                }
            });
    }

    void trigger_cameras() {
        // Execute the PWM commands to trigger cameras
        RCLCPP_INFO(this->get_logger(), "Starting PWM signal to trigger cameras...");

        std::system("echo 0 > /sys/class/pwm/pwmchip0/export");
        std::system("echo 66666666 > /sys/class/pwm/pwmchip0/pwm0/period");
        std::system("echo 33333333 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle");
        std::system("echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable");

        RCLCPP_INFO(this->get_logger(), "PWM signal triggered at ROS time: %f.%09ld",
                    trigger_time_.seconds(), trigger_time_.nanoseconds());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoTrigNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
