from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = []

def generate_launch_description():

    # Get the package share directory
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    
    # Launch Gazebo with TurtleBot 4
    gazebo_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_ignition.launch.py'])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
    )

    # Define custom nodes for Lava integration
    lidar_to_lava_node = Node(
        package='turtlebot4_lava_integration',
        executable='lidar_to_lava',
        name='lidar_to_lava',
        output='screen'
    )

    lava_output_to_cmd_node = Node(
        package='turtlebot4_lava_integration',
        executable='lava_output_to_cmd',
        name='lava_output_to_cmd',
        output='screen'
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(lidar_to_lava_node)
    ld.add_action(lava_output_to_cmd_node)
    return ld
