#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class GoToPose : public rclcpp::Node
{
public:
    using GoToPoseAction = robot_patrol::action::GoToPose;
    using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;
    
    explicit GoToPose() : Node("go_to_pose_node")
    {
        // Create the action server
        this->action_server_ = rclcpp_action::create_server<GoToPoseAction>(
            this,
            "/go_to_pose",
            std::bind(&GoToPose::handle_goal, this, _1, _2),
            std::bind(&GoToPose::handle_cancel, this, _1),
            std::bind(&GoToPose::handle_accepted, this, _1));
        
        // Create a subscriber for odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&GoToPose::odom_callback, this, _1));
        
        // Create a publisher for velocity commands
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Initialize the current position
        current_pos_.x = 0.0;
        current_pos_.y = 0.0;
        current_pos_.theta = 0.0;
        
        // Initialize the desired position
        desired_pos_.x = 0.0;
        desired_pos_.y = 0.0;
        desired_pos_.theta = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Action Server Ready");
    }

private:
    // The action server
    rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
    
    // Subscriber for odometry data
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // Publisher for velocity commands
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    
    // Current position of the robot
    geometry_msgs::msg::Pose2D current_pos_;
    
    // Desired position for the robot
    geometry_msgs::msg::Pose2D desired_pos_;
    
    // Timer for the control loop
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Callback function for the odometry data
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Update current position from odometry
        current_pos_.x = msg->pose.pose.position.x;
        current_pos_.y = msg->pose.pose.position.y;
        
        // Convert quaternion to Euler angles to get the theta
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        
        // Extract yaw angle (theta)
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // Convert to degrees if needed (the desired_pos_.theta is in degrees)
        current_pos_.theta = yaw * 180.0 / M_PI;
    }
    
    // Function to handle goal requests
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GoToPoseAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request: x=%f, y=%f, theta=%f",
                   goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
        
        (void)uuid;
        
        // Always accept goals
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    // Function to handle cancel requests
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        
        (void)goal_handle;
        
        // Always allow cancellation
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    // Function to handle accepted goals
    void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Action Called");
        
        // Store the goal position
        desired_pos_ = goal_handle->get_goal()->goal_pos;
        
        // Create a timer for the control loop at 10 Hz
        timer_ = this->create_wall_timer(
            100ms, [this, goal_handle]() { control_loop(goal_handle); });
    }
    
    // Control loop function
    void control_loop(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        // Check if the goal is still active
        if (goal_handle->is_canceling()) {
            // Stop the robot
            stop_robot();
            
            // Set the result
            auto result = std::make_shared<GoToPoseAction::Result>();
            result->status = false;
            
            // Mark the goal as canceled
            goal_handle->canceled(result);
            
            // Stop the timer
            timer_->cancel();
            
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        
        // Calculate distance to goal
        double dx = desired_pos_.x - current_pos_.x;
        double dy = desired_pos_.y - current_pos_.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        // Calculate angle to goal
        double target_angle = std::atan2(dy, dx) * 180.0 / M_PI;  // Convert to degrees
        
        // Calculate angle difference
        double angle_diff = target_angle - current_pos_.theta;
        
        // Normalize angle difference to be between -180 and 180 degrees
        while (angle_diff > 180.0) angle_diff -= 360.0;
        while (angle_diff < -180.0) angle_diff += 360.0;
        
        // Create velocity command message
        auto vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
        
        // Check if we've reached the goal
        if (distance < 0.1 && std::abs(angle_diff) < 5.0) {
            // Stop the robot
            stop_robot();
            
            // Set the result
            auto result = std::make_shared<GoToPoseAction::Result>();
            result->status = true;
            
            // Mark the goal as succeeded
            goal_handle->succeed(result);
            
            // Stop the timer
            timer_->cancel();
            
            RCLCPP_INFO(this->get_logger(), "Action Completed");
            return;
        }
        
        // Publish feedback
        auto feedback = std::make_shared<GoToPoseAction::Feedback>();
        feedback->current_pos = current_pos_;
        goal_handle->publish_feedback(feedback);
        
        // Set angular velocity proportional to angle difference
        vel_msg->angular.z = 0.5 * angle_diff / 180.0;  // Scale to a reasonable range
        
        // Limit angular velocity
        if (vel_msg->angular.z > 0.5) vel_msg->angular.z = 0.5;
        if (vel_msg->angular.z < -0.5) vel_msg->angular.z = -0.5;
        
        // Set linear velocity based on angle alignment
        // Only move forward if we're mostly pointing in the right direction
        if (std::abs(angle_diff) < 30.0) {
            vel_msg->linear.x = 0.2 * (1.0 - std::abs(angle_diff) / 30.0); // Scale down as angle increases
        } else {
            vel_msg->linear.x = 0.0;  // Don't move forward when angle is too large
        }
        
        // Publish the velocity command
        vel_pub_->publish(std::move(vel_msg));
    }
    
    // Function to stop the robot
    void stop_robot()
    {
        auto vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
        vel_msg->linear.x = 0.0;
        vel_msg->linear.y = 0.0;
        vel_msg->linear.z = 0.0;
        vel_msg->angular.x = 0.0;
        vel_msg->angular.y = 0.0;
        vel_msg->angular.z = 0.0;
        vel_pub_->publish(std::move(vel_msg));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToPose>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}