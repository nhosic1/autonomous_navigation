from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = "autonomous_navigation"
    share_dir = get_package_share_directory(package_name)

    map_yaml_path = os.path.join(share_dir, "config", "map.yaml")
    nav2_params_path = os.path.join(share_dir, "config", "nav2_params_iron.yaml")
    nav_to_pose_bt_xml_path = os.path.join(
        share_dir, "behavior_trees", "replan_on_goal_update_or_obstacle.xml"
    )

    lifecycle_nodes = [
        "map_server",
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

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
                package="nav2_map_server",
                executable="map_server",
                output="screen",
                parameters=[nav2_params_path, {'yaml_filename': map_yaml_path}, {"use_sim_time": sim}],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                remappings=[("cmd_vel", "cmd_vel_nav")],
                parameters=[nav2_params_path, {"use_sim_time": sim}],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                output="screen",
                parameters=[nav2_params_path, {"use_sim_time": sim}],
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_params_path, {"use_sim_time": sim}],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                output="screen",
                parameters=[nav2_params_path, {"default_nav_to_pose_bt_xml": nav_to_pose_bt_xml_path, "use_sim_time": sim}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                output="screen",
                parameters=[{"autostart": True, "node_names": lifecycle_nodes, "use_sim_time": sim}],
            ),
        ]
    )
