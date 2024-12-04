#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <string>
#include <cstdlib> // for std::system
#include <memory>  // for std::shared_ptr
#include <cmath>   // for ceil
#include "vimbax_camera_msgs/srv/trigger_time.hpp" // Include your custom service definition

using namespace std::chrono_literals;

class StereoTrigNode : public rclcpp::Node {
public:
    StereoTrigNode() : Node("stereo_trig_node") {
        // Declare the FPS parameter with a default value
        this->declare_parameter<int>("fps", 15);

        // Get the FPS parameter value
        this->get_parameter("fps", fps_);

        // Initialize parameters
        trigger_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

        // Service clients
        left_camera_client_ = this->create_client<vimbax_camera_msgs::srv::TriggerTime>("/vimbax_camera_left/set_trigger_time");
        right_camera_client_ = this->create_client<vimbax_camera_msgs::srv::TriggerTime>("/vimbax_camera_right/set_trigger_time");

        // Initialize synchronization process
        send_trigger_time(1s);
    }

    ~StereoTrigNode() {
        // Cleanup PWM signal if it was exported
        std::string command = "sudo /usr/local/bin/pwm_control.sh disable";
        int ret = std::system(command.c_str());
        if (ret == 0) {
            RCLCPP_INFO(this->get_logger(), "PWM signal disabled.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to disable PWM signal. Command returned: %d", ret);
        }
    }


private:
    rclcpp::Time trigger_time_;
    rclcpp::Client<vimbax_camera_msgs::srv::TriggerTime>::SharedPtr left_camera_client_;
    rclcpp::Client<vimbax_camera_msgs::srv::TriggerTime>::SharedPtr right_camera_client_;
    int fps_;

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
        auto left_future = left_camera_client_->async_send_request(request);
        auto right_future = right_camera_client_->async_send_request(request);

        // Wait for both responses to be completed
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), left_future);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), right_future);

        bool left_ack = left_future.get()->success;
        bool right_ack = right_future.get()->success;

        // Check for acknowledgments
        if (left_ack && right_ack) {
            RCLCPP_INFO(this->get_logger(), "Both cameras acknowledged the trigger time.");

            // Wait until the trigger time
            rclcpp::Time now = this->get_clock()->now();
            while (now < trigger_time_) {
                // Calculate the remaining time
                rclcpp::Duration remaining_time = trigger_time_ - now;

                // Sleep for a small fraction of the remaining time
                rclcpp::Duration sleep_duration = std::min(remaining_time, rclcpp::Duration::from_seconds(0.01));
                rclcpp::sleep_for(std::chrono::nanoseconds(sleep_duration.nanoseconds()));

                // Update current time
                now = this->get_clock()->now();
            }

            // Trigger the cameras at the exact trigger time
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
    }

    void trigger_cameras() {
        // Calculate the period and duty cycle in nanoseconds
        // Period = 1 / fps * 1e9 to get nanoseconds
        int64_t period_ns = static_cast<int64_t>(std::ceil((1.0 / fps_) * 1e9));
        int64_t duty_cycle_ns = period_ns / 2;  // Duty cycle is half the period

        // Use the PWM control script with sudo to enable PWM
        std::string command = "sudo /usr/local/bin/pwm_control.sh enable " + std::to_string(period_ns) + " " + std::to_string(duty_cycle_ns);
        int ret = std::system(command.c_str());
        if (ret == 0) {
            RCLCPP_INFO(this->get_logger(), "PWM signal triggered at FPS: %d (period: %ld ns, duty cycle: %ld ns)", fps_, period_ns, duty_cycle_ns);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to trigger PWM signal. Command returned: %d", ret);
        }
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoTrigNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
