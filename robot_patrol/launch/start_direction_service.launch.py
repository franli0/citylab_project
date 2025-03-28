from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('robot_patrol')
    
    # Create the direction service node launch configuration
    direction_service_node = Node(
        package='robot_patrol',
        executable='direction_service',
        name='direction_service_node',
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        direction_service_node
    ])