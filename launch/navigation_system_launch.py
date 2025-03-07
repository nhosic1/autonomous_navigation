import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_dir = get_package_share_directory("autonomous_navigation")
    model_path = os.path.join(package_share_dir, "models", "autonomous_vehicle", "model.urdf")

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


    return LaunchDescription([
        sim_arg,
        data_folder_arg,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen"
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            arguments=[model_path],
        ),
        TimerAction(
            period=0.2,
            actions=[
                Node(
                    package="autonomous_navigation",
                    executable="autonomous_navigator",
                    parameters=[{"sim": sim, "data_folder": data_folder}],
                )
            ],
        )
    ])