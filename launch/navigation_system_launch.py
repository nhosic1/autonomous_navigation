import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share_dir = get_package_share_directory("autonomous_navigation")
    model_path = os.path.join(
        package_share_dir, "models", "autonomous_vehicle", "model.urdf"
    )
    ekf_config_path = os.path.join(package_share_dir, "config", "ekf_localization.yaml")

    sim = LaunchConfiguration("sim")
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description="Enable simulation-specific stereo camera configuration.",
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
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        arguments=[model_path],
    )

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        # arguments=[
        #     "--x",
        #     "0.35",
        #     "--y",
        #     "0.09",
        #     "--z",
        #     "0.35",
        #     "--yaw",
        #     "0",
        #     "--pitch",
        #     "0",
        #     "--roll",
        #     "0",
        #     "--frame-id",
        #     "odom",
        #     "--child-frame-id",
        #     "odom_visual",
        # ],
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--yaw",
            "0",
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            "odom",
            "--child-frame-id",
            "base_link",
        ],
    )

    vo_estimator = Node(
        package="autonomous_navigation",
        executable="vo_estimator",
        parameters=[{"sim": sim, "data_folder": data_folder}],
    )

    imu_covariance_fixer = Node(
            package="autonomous_navigation",
            executable="imu_covariance_fixer.py",
            name="imu_covariance_fixer",
            output="screen"
        )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_path],
    )

    return LaunchDescription(
        [
            sim_arg,
            data_folder_arg,
            robot_state_publisher,
            joint_state_publisher,
            imu_covariance_fixer,
            # static_transform_publisher,
            TimerAction(
                period=0.2,
                actions=[
                    vo_estimator,
                    ekf_node,
                ],
            ),
        ]
    )
