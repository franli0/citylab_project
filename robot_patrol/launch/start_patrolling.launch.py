from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('robot_patrol')
    
    # Create path to rviz config file
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'robot_patrol.rviz')
    
    # Declare the rviz config file path as a launch argument with a default value
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_path,
        description='Path to RViz config file'
    )
    
    # Create the patrol node launch configuration
    patrol_node = Node(
        package='robot_patrol',
        executable='patrol',
        name='patrol_node',
        output='screen'
    )
    
    # Create the RViz node launch configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        rviz_config_arg,
        patrol_node,
        rviz_node
    ])