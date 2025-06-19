import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share_dir = get_package_share_directory("autonomous_navigation")
    rviz_config_path = os.path.join(
        package_share_dir, "config", "autonomous_navigation.rviz"
    )

    sim = LaunchConfiguration("sim")
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description="Enable simulation-specific configurations.",
    )

    return LaunchDescription(
        [
            sim_arg,
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_path],
                parameters=[{"use_sim_time": sim}],
            ),
        ]
    )
