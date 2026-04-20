from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = "autonomous_navigation"
    package_share_dir = get_package_share_directory(package_name)
    ros_gz_sim_share_dir = get_package_share_directory("ros_gz_sim")
    config_path = os.path.join(package_share_dir, "config", "ros_gz_bridge_config.yaml")
    model_path = os.path.join(package_share_dir, "models", "autonomous_vehicle", "model.urdf")
    world_path = os.path.join(
        package_share_dir, "worlds", "warehouse_world.sdf"
    )

    world = LaunchConfiguration("world")
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=world_path,
        description="Absolute path to a world file to open",
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[
            ("gz_args", world),
        ],
    )

    sensor_message_normalizer = Node(
        package=package_name,
        executable="sensor_message_normalizer",
        name="sensor_message_normalizer",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            world_arg,
            gazebo_sim,
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-file", model_path, "-x", "0.3", "-z", "0.15"],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                output="screen",
                ros_arguments=["-p", f"config_file:={config_path}"],
            ),
            sensor_message_normalizer,
        ]
    )
