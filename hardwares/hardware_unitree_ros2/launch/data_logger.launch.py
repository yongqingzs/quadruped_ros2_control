#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    log_flag_arg = DeclareLaunchArgument(
        'log_flag',
        default_value='true',
        description='Enable data logging'
    )
    
    logging_duration_arg = DeclareLaunchArgument(
        'logging_duration',
        default_value='3',
        description='Duration of data logging in seconds'
    )

    # Data Logger Node
    data_logger_node = Node(
        package='hardware_unitree_ros2',
        executable='data_logger_node',
        name='data_logger_node',
        output='screen',
        parameters=[{
            'log_flag': LaunchConfiguration('log_flag'),
            'logging_duration': LaunchConfiguration('logging_duration'),
        }]
    )

    return LaunchDescription([
        log_flag_arg,
        logging_duration_arg,
        data_logger_node,
    ])
