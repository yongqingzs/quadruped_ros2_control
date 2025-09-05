#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'network_interface',
            default_value='lo',
            description='Network interface for DDS communication'
        ),
        DeclareLaunchArgument(
            'domain',
            default_value='1',
            description='DDS domain ID'
        ),
        DeclareLaunchArgument(
            'debug_output',
            default_value='false',
            description='Enable debug output'
        ),
        
        # DDS-ROS2 Bridge Node
        Node(
            package='hardware_unitree_ros2',
            executable='dds_ros2_bridge_node',
            name='dds_ros2_bridge',
            output='screen',
            parameters=[{
                'network_interface': LaunchConfiguration('network_interface'),
                'domain': LaunchConfiguration('domain'),
                'debug_output': LaunchConfiguration('debug_output'),
            }],
            remappings=[
                # You can add topic remappings here if needed
            ]
        ),
    ])
