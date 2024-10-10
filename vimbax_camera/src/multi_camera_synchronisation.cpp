#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/header.hpp"

class MasterCameraSubscriber : public rclcpp::Node
{
public:
    MasterCameraSubscriber():
        Node("master_camera_subscriber"),
        stamp_(0)
    {
        initialize_subscription();
        stamp_ = 0;
    }

    void initialize_subscription() 
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/vimbax_camera_left/camera_info", 10, std::bind(&MasterCameraSubscriber::topic_callback, this, std::placeholders::_1));
    }

    void topic_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) 
    {
        stamp_ = static_cast<double>(msg->header.stamp.nanosec);
    }

    bool has_received_message()
    {
        if (stamp_ > 0) {
            return true;
        }
        return false;
    }

    // Variables
    int stamp_;

private:
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_;
};

class SlaveCameraSubscriber : public rclcpp::Node
{
public:
    SlaveCameraSubscriber():
        Node("slave_camera_subscriber"),
        stamp_(0)
    {
        initialize_subscription();
        stamp_ = 0;
    }

    void initialize_subscription() 
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/vimbax_camera_right/camera_info", 10, std::bind(&SlaveCameraSubscriber::topic_callback, this, std::placeholders::_1));
    }

    void topic_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) 
    {
        stamp_ = static_cast<double>(msg->header.stamp.nanosec);
    }

    bool has_received_message()
    {
        if (stamp_ > 0.0) {
            return true;
        }
        return false;
    }

    // Variables
    int stamp_;

private:
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_;
};

class NodeManager
{
public:
    NodeManager(double time_threshold) : time_threshold_(time_threshold)
    {
        create_nodes();

        diff_ = 0;
        recreateCounter_ = 3;
        ready_ = false;
    }

    void create_nodes()
    {
        create_master_node();
        create_slave_node();
        return;
    }

    void create_master_node()
    {
        masterNode_ = std::make_shared<MasterCameraSubscriber>();
        RCLCPP_INFO(masterNode_->get_logger(), "Master node created");
        return ;
    }

    void create_slave_node()
    {
        slaveNode_ = std::make_shared<SlaveCameraSubscriber>();
        RCLCPP_INFO(slaveNode_->get_logger(), "Slave node created");
        return;
    }

    // Method to delete and recreate the node
    void recreate_slave_node()
    {
        // Delete the node (reset the shared pointer)
        if (slaveNode_) {
            RCLCPP_INFO(slaveNode_->get_logger(), "Deleting node...");
            slaveNode_.reset();  // This deletes the node
            RCLCPP_INFO(rclcpp::get_logger("NodeManager"), "Node deleted");
        }

        // Recreate the node
        create_slave_node();
        recreateCounter_ = 3;

        return;
    }

    // Spin the node while it exists
    void spin_nodes()
    {
        if (!ready_) {
            rclcpp::spin_some(masterNode_);
            rclcpp::spin_some(slaveNode_);

            if (masterNode_->has_received_message() && slaveNode_->has_received_message()) {
                // Calculate the difference between the two timestamps
                diff_ = (masterNode_->stamp_ - slaveNode_->stamp_)/1000000.0;
                RCLCPP_INFO(masterNode_->get_logger(), "Time difference in ms: '%f'", diff_);
                
                if (diff_ < time_threshold_ && diff_ > -time_threshold_) {
                    ready_ = true;
                    RCLCPP_INFO(masterNode_->get_logger(), "Ready");
                    RCLCPP_INFO(masterNode_->get_logger(), "Time difference in ms: '%f'", diff_);
                } 

                if (!ready_){
                    RCLCPP_INFO(masterNode_->get_logger(), "Not ready");
                    if ((diff_ > time_threshold_ || diff_ < -time_threshold_) && recreateCounter_ < 0) {
                        RCLCPP_INFO(masterNode_->get_logger(), "Reset subscription to slave node");
                        recreate_slave_node();
                    }
                }

                recreateCounter_--;
            }
        }
    }

private:
    std::shared_ptr<MasterCameraSubscriber> masterNode_;
    std::shared_ptr<SlaveCameraSubscriber> slaveNode_;

    double diff_;
    int recreateCounter_;
    bool ready_;
    double time_threshold_;  // Add time threshold
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Check for the time threshold argument from the command line
    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: %s <time_threshold>", argv[0]);
        return 1;
    }

    // Convert the command line argument to a double value for time threshold
    double time_threshold = std::stod(argv[1]);

    // Create an instance of the node manager with the given time threshold
    NodeManager manager(time_threshold);

    // Spin to handle callbacks and node recreation
    rclcpp::Rate loop_rate(100); 
    while (rclcpp::ok()) {
        manager.spin_nodes();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
