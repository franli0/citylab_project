#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <vector>
#include <cmath>
#include <memory>
#include <limits>

using namespace std::chrono_literals;

class PatrolWithService : public rclcpp::Node
{
public:
    PatrolWithService() : Node("patrol_node")
    {
        // Create a subscriber for the laser scan topic
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PatrolWithService::laser_callback, this, std::placeholders::_1));
        
        // Create a publisher for the velocity commands
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
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
        
        // Initialize variables
        obstacle_front_ = false;
        direction_ = "forward";
        
        RCLCPP_INFO(this->get_logger(), "Patrol With Service node initialized");
    }

private:
    // Subscriber for laser scan data
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    
    // Publisher for velocity commands
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    
    // Client for the direction service
    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
    
    // Flag to indicate if an obstacle is detected in front
    bool obstacle_front_;
    
    // Direction to move
    std::string direction_;
    
    // Laser data for processing
    sensor_msgs::msg::LaserScan latest_scan_;
    
    // Callback function for the laser scan data
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Store the latest scan
        latest_scan_ = *msg;
        
        // Check if there is an obstacle in front of the robot (< 35cm)
        obstacle_front_ = false;
        float min_front_distance = std::numeric_limits<float>::infinity();
        
        // Get the number of laser rays
        int num_rays = msg->ranges.size();
        
        // Get the angle increment between rays
        float angle_increment = msg->angle_increment;
        
        // Find the indices for front area
        int middle_index = num_rays / 2;  // This should be the ray pointing directly forward
        int front_range = static_cast<int>(M_PI/6 / angle_increment);  // 30 degree arc in front
        int front_start = middle_index - front_range;
        int front_end = middle_index + front_range;
        
        // Make sure indices are within bounds
        front_start = std::max(0, front_start);
        front_end = std::min(num_rays - 1, front_end);
        
        // Check front distance
        for (int i = front_start; i <= front_end; ++i) {
            float range = msg->ranges[i];
            if (std::isfinite(range) && range < min_front_distance) {
                min_front_distance = range;
            }
        }
        
        // Use the 35cm threshold for obstacle detection
        if (min_front_distance < 0.35) {
            obstacle_front_ = true;
            RCLCPP_INFO(this->get_logger(), "Obstacle detected at %f meters", min_front_distance);
            
            // Call the service when an obstacle is detected
            call_direction_service();
        } else {
            // Default to moving forward when no obstacle
            direction_ = "forward";
            move_robot();
        }
    }
    
    // Function to call the direction service
    void call_direction_service()
    {
        auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
        request->laser_data = latest_scan_;
        
        auto future = client_->async_send_request(
            request,
            std::bind(&PatrolWithService::response_callback, this, std::placeholders::_1)
        );
    }
    
    // Callback function for the service response
    void response_callback(rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future)
    {
        auto response = future.get();
        direction_ = response->direction;
        RCLCPP_INFO(this->get_logger(), "Service Response: %s", direction_.c_str());
        
        // Move the robot based on the direction
        move_robot();
    }
    
    // Function to move the robot
    void move_robot()
    {
        // Create velocity command message
        auto vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
        
        // Set default values
        vel_msg->linear.y = 0.0;
        vel_msg->linear.z = 0.0;
        vel_msg->angular.x = 0.0;
        vel_msg->angular.y = 0.0;
        
        // Set velocities based on direction
        if (direction_ == "forward") {
            vel_msg->linear.x = 0.1;
            vel_msg->angular.z = 0.0;
        } else if (direction_ == "left") {
            vel_msg->linear.x = 0.1;
            vel_msg->angular.z = 0.5;
        } else if (direction_ == "right") {
            vel_msg->linear.x = 0.1;
            vel_msg->angular.z = -0.5;
        }
        
        // Publish the velocity command
        vel_pub_->publish(std::move(vel_msg));
        
        RCLCPP_DEBUG(this->get_logger(), "Publishing velocity - linear: %f, angular: %f", 
                   vel_msg->linear.x, vel_msg->angular.z);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolWithService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}