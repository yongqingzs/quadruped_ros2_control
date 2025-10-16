from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_controller = "ocs2_quadruped_controller"

def generate_launch_description():
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(package_controller),
            "config",
            "visualize_ocs2.rviz",
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_ocs2',
        output='screen',
        arguments=["-d", rviz_config_file]
    )

    return LaunchDescription([
        rviz,
    ])