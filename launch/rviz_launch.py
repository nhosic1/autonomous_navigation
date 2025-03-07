import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_dir = get_package_share_directory("autonomous_navigation")
    model_path = os.path.join(package_share_dir, "models", "autonomous_vehicle", "model.urdf")
    rviz_config_path = os.path.join(package_share_dir, "config", "autonomous_navigation.rviz")

    return LaunchDescription([
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            arguments=[model_path],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--x", "0.35", "--y", "0.09", "--z", "0.35", "--yaw", "0", "--pitch", "0", "--roll", "0", "--frame-id", "world", "--child-frame-id", "odom"]
        ),
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     arguments=["--x", "0", "--y", "0", "--z", "0", "--yaw", "0", "--pitch", "0", "--roll", "0", "--frame-id", "world", "--child-frame-id", "odom"]
        # ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_path],
        )
    ])