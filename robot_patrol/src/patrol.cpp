#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node
{
public:
    Patrol() : Node("patrol_node")
    {
        // Create a subscriber for the laser scan topic
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Patrol::laser_callback, this, std::placeholders::_1));
        
        // Create a publisher for the velocity commands
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Create a timer for the control loop at 10 Hz
        timer_ = this->create_wall_timer(100ms, std::bind(&Patrol::control_loop, this));
        
        // Initialize the direction variable
        direction_ = 0.0;
        
        // Initialize the obstacle detected flag
        obstacle_front_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Patrol node initialized");
    }

private:
    // Subscriber for laser scan data
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    
    // Publisher for velocity commands
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    
    // Timer for the control loop
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Direction to the safest area (angle)
    float direction_;
    
    // Flag to indicate if an obstacle is detected in front
    bool obstacle_front_;
    
    // Callback function for the laser scan data
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Get the number of laser rays
        int num_rays = msg->ranges.size();
        
        // Get the angle increment between rays
        float angle_increment = msg->angle_increment;
        
        // Find the indices for front 180 degrees
        int middle_index = num_rays / 2;  // This should be the ray pointing directly forward
        int range_180 = static_cast<int>(M_PI / angle_increment);  // Number of rays in 180 degrees
        int start_index = middle_index - range_180/2;  // Start of 180 degree arc
        int end_index = middle_index + range_180/2;    // End of 180 degree arc
        
        // Make sure indices are within bounds
        start_index = std::max(0, start_index);
        end_index = std::min(num_rays - 1, end_index);
        
        // Check if there is an obstacle in front of the robot (< 35cm)
        obstacle_front_ = false;
        float min_front_distance = std::numeric_limits<float>::infinity();
        
        // Define a narrower frontal area to check for obstacles
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
        
        RCLCPP_DEBUG(this->get_logger(), "Min front distance: %f", min_front_distance);
        
        // Use the original 35cm threshold for obstacle detection
        if (min_front_distance < 0.35) {
            obstacle_front_ = true;
            RCLCPP_INFO(this->get_logger(), "Obstacle detected at %f meters", min_front_distance);
        }
        
        // Always scan for the safest direction to improve responsiveness
        // Find the largest distance ray within the front 180 degrees
        float max_distance = 0.0;
        int max_index = middle_index;  // Default to front
        
        for (int i = start_index; i <= end_index; ++i) {
            float range = msg->ranges[i];
            if (std::isfinite(range) && range > max_distance) {
                max_distance = range;
                max_index = i;
            }
        }
        
        // Calculate the angle corresponding to the largest distance ray
        float angle = msg->angle_min + max_index * angle_increment;
        
        // Ensure the angle is between -pi/2 and pi/2
        while (angle > M_PI/2) {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI/2) {
            angle += 2 * M_PI;
        }
        
        // Adjust turning behavior based on obstacle proximity
        if (obstacle_front_) {
            // When obstacle detected at close range, make a pronounced turn
            direction_ = angle;
            RCLCPP_INFO(this->get_logger(), "Obstacle close! Turning toward: %f radians", direction_);
        } else if (min_front_distance < 0.5) {
            // If obstacle is nearby but not critically close, make a gentler turn
            // Use a proportional approach: turn more sharply as obstacles get closer
            float turn_factor = 0.8 * (1.0 - (min_front_distance / 0.5));
            direction_ = angle * turn_factor;
            RCLCPP_INFO(this->get_logger(), "Obstacle approaching at %f meters. Gentle turn: %f radians", 
                        min_front_distance, direction_);
        } else {
            // Gradually return to forward when no obstacle
            direction_ = direction_ * 0.5;  // Faster decay to straight path
        }
    }
    
    // Control loop function
    void control_loop()
    {
        // Create velocity command message
        auto vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
        
        // Adjust linear velocity based on obstacle presence
        if (obstacle_front_) {
            // Slow down when obstacle detected very close
            vel_msg->linear.x = 0.05;  // Slower forward speed
        } else {
            // Normal speed when no immediate obstacle
            vel_msg->linear.x = 0.1;
        }
        vel_msg->linear.y = 0.0;
        vel_msg->linear.z = 0.0;
        
        // Set angular velocity based on the direction
        vel_msg->angular.x = 0.0;
        vel_msg->angular.y = 0.0;
        vel_msg->angular.z = direction_ / 2.0;  // Back to original value for more gradual turning
        
        // Publish the velocity command
        vel_pub_->publish(std::move(vel_msg));
        
        RCLCPP_DEBUG(this->get_logger(), "Publishing velocity - linear: %f, angular: %f", 
                   vel_msg->linear.x, vel_msg->angular.z);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}