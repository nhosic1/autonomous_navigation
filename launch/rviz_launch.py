import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share_dir = get_package_share_directory("autonomous_navigation")
    rviz_config_path = os.path.join(
        package_share_dir, "config", "autonomous_navigation.rviz"
    )

    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_path],
            ),
        ]
    )
