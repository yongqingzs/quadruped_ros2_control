#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "connection_settings",
            default_value="LOW_WIRED_DEFAULTS",
            description="Free Dog SDK connection settings",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "show_foot_force",
            default_value="false",
            description="Whether to show foot force in logs",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_high_level",
            default_value="false",
            description="Whether to use high-level state data",
        )
    )

    # Initialize Arguments
    connection_settings = LaunchConfiguration("connection_settings")
    show_foot_force = LaunchConfiguration("show_foot_force")
    use_high_level = LaunchConfiguration("use_high_level")

    # Robot description
    robot_description_content = """
    <robot name="free_dog">
      <ros2_control name="free_dog_hardware" type="system">
        <hardware>
          <plugin>HardwareFreeDogSdk</plugin>
          <param name="connection_settings">{connection_settings}</param>
          <param name="show_foot_force">{show_foot_force}</param>
          <param name="use_high_level">{use_high_level}</param>
        </hardware>
        <!-- Joint definitions would go here -->
      </ros2_control>
    </robot>
    """.format(
        connection_settings=connection_settings,
        show_foot_force=show_foot_force,
        use_high_level=use_high_level
    )

    robot_description = {"robot_description": robot_description_content}

    # Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description],
        output="both",
    )

    nodes = [
        control_node,
    ]

    return LaunchDescription(declared_arguments + nodes)