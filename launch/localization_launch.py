import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "autonomous_navigation"
    package_share_dir = get_package_share_directory(package_name)
    model_path = os.path.join(
        package_share_dir, "models", "autonomous_vehicle", "model.urdf"
    )
    ekf_config_path = os.path.join(package_share_dir, "config", "ekf_localization.yaml")

    sim = LaunchConfiguration("sim")
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description="Enable simulation-specific configurations.",
    )

    data_folder = LaunchConfiguration("data_folder")
    data_folder_arg = DeclareLaunchArgument(
        "data_folder",
        default_value="",
        description="Set path to data folder for saving images on navigation failure.",
    )

    with open(model_path, "r") as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": sim}],
        output="screen",
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        arguments=[model_path],
        parameters=[{"use_sim_time": sim}],
    )

    wheel_odom_estimator = Node(
        package=package_name,
        executable="wheel_odom_estimator",
        parameters=[{"use_sim_time": sim}],
    )

    visual_odom_estimator = Node(
        package=package_name,
        executable="visual_odom_estimator",
        parameters=[{"sim": sim, "data_folder": data_folder, "use_sim_time": sim}],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_path, {"use_sim_time": sim}],
    )

    localization_initializer = Node(
        package=package_name,
        executable="localization_initializer",
        output="screen",
        parameters=[{"use_sim_time": sim}],
    )

    return LaunchDescription(
        [
            sim_arg,
            data_folder_arg,
            robot_state_publisher,
            # joint_state_publisher,
            TimerAction(
                period=0.2,
                actions=[
                    wheel_odom_estimator,
                    visual_odom_estimator,
                    ekf_node,
                    localization_initializer,
                ],
            ),
        ]
    )
