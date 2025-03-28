#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>

class TestService : public rclcpp::Node
{
public:
    TestService() : Node("test_service_node")
    {
        // Create a subscriber for the laser scan topic
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&TestService::laser_callback, this, std::placeholders::_1));
        
        // Create a client for the direction service
        client_ = this->create_client<robot_patrol::srv::GetDirection>("/direction_service");
        
        // Wait for the service to be available
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
        }
        
        RCLCPP_INFO(this->get_logger(), "Service Client Ready");
    }

private:
    // Subscriber for laser scan data
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    
    // Client for the direction service
    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
    
    // Callback function for the laser scan data
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Create a request with the laser data
        auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
        request->laser_data = *msg;
        
        RCLCPP_INFO(this->get_logger(), "Service Request");
        
        // Send the service request
        auto future = client_->async_send_request(
            request,
            std::bind(&TestService::response_callback, this, std::placeholders::_1)
        );
    }
    
    // Callback function for the service response
    void response_callback(rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Service Response: %s", response->direction.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}