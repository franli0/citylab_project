from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('robot_patrol')
    
    # Create the test service node launch configuration
    test_service_node = Node(
        package='robot_patrol',
        executable='test_service',
        name='test_service_node',
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        test_service_node
    ])