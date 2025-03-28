#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <memory>
#include <limits>

class DirectionService : public rclcpp::Node
{
public:
    DirectionService() : Node("direction_service_node")
    {
        // Create the service server
        service_ = this->create_service<robot_patrol::srv::GetDirection>(
            "/direction_service",
            std::bind(&DirectionService::analyze_laser, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        RCLCPP_INFO(this->get_logger(), "Service Server Ready");
    }

private:
    // Service for analyzing laser data and providing direction
    rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;

    void analyze_laser(
        const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
        std::shared_ptr<robot_patrol::srv::GetDirection::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Service Requested");
        
        // Get laser data from request
        const auto& laser_data = request->laser_data;
        
        // Divide the laser rays into 3 sections of 60° each
        int num_rays = laser_data.ranges.size();
        float angle_increment = laser_data.angle_increment;
        
        // Find the indices for front 180 degrees
        int middle_index = num_rays / 2;  // This should be the ray pointing directly forward
        
        // Calculate the number of rays in a 60° section
        int rays_per_section = static_cast<int>(M_PI/3.0 / angle_increment);  // 60 degrees = pi/3 radians
        
        // Define the sections
        int right_start = middle_index - (rays_per_section * 3/2);  // Start of right section
        int right_end = middle_index - (rays_per_section * 1/2);    // End of right section
        
        int front_start = middle_index - (rays_per_section * 1/2);  // Start of front section
        int front_end = middle_index + (rays_per_section * 1/2);    // End of front section
        
        int left_start = middle_index + (rays_per_section * 1/2);   // Start of left section
        int left_end = middle_index + (rays_per_section * 3/2);     // End of left section
        
        // Ensure indices are within bounds
        right_start = std::max(0, right_start);
        right_end = std::min(num_rays - 1, right_end);
        front_start = std::max(0, front_start);
        front_end = std::min(num_rays - 1, front_end);
        left_start = std::max(0, left_start);
        left_end = std::min(num_rays - 1, left_end);
        
        // Calculate total distances for each section
        float total_dist_sec_right = 0.0;
        float total_dist_sec_front = 0.0;
        float total_dist_sec_left = 0.0;
        
        // Sum distances for right section
        for (int i = right_start; i <= right_end; ++i) {
            float range = laser_data.ranges[i];
            if (std::isfinite(range)) {
                total_dist_sec_right += range;
            }
        }
        
        // Sum distances for front section
        for (int i = front_start; i <= front_end; ++i) {
            float range = laser_data.ranges[i];
            if (std::isfinite(range)) {
                total_dist_sec_front += range;
            }
        }
        
        // Sum distances for left section
        for (int i = left_start; i <= left_end; ++i) {
            float range = laser_data.ranges[i];
            if (std::isfinite(range)) {
                total_dist_sec_left += range;
            }
        }
        
        // Check if front distance is above the threshold (35 cm)
        float min_front_distance = std::numeric_limits<float>::infinity();
        for (int i = front_start; i <= front_end; ++i) {
            float range = laser_data.ranges[i];
            if (std::isfinite(range) && range < min_front_distance) {
                min_front_distance = range;
            }
        }
        
        // Default direction is forward
        std::string direction = "forward";
        
        // If front distance is less than 35 cm, choose another direction
        if (min_front_distance < 0.35) {
            // Determine which section has the largest total distance
            if (total_dist_sec_right >= total_dist_sec_left && total_dist_sec_right >= total_dist_sec_front) {
                direction = "right";
                RCLCPP_INFO(this->get_logger(), "Choosing RIGHT - distances: R=%.2f, F=%.2f, L=%.2f", 
                          total_dist_sec_right, total_dist_sec_front, total_dist_sec_left);
            } else if (total_dist_sec_left >= total_dist_sec_right && total_dist_sec_left >= total_dist_sec_front) {
                direction = "left";
                RCLCPP_INFO(this->get_logger(), "Choosing LEFT - distances: R=%.2f, F=%.2f, L=%.2f", 
                          total_dist_sec_right, total_dist_sec_front, total_dist_sec_left);
            } else {
                direction = "forward";
                RCLCPP_INFO(this->get_logger(), "Choosing FORWARD - distances: R=%.2f, F=%.2f, L=%.2f", 
                          total_dist_sec_right, total_dist_sec_front, total_dist_sec_left);
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "No obstacle, continuing FORWARD. Min front distance: %.2f", min_front_distance);
        }
        
        // Set response
        response->direction = direction;
        
        RCLCPP_INFO(this->get_logger(), "Service Completed");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectionService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}