from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'your_package'  # change this
    share_dir = get_package_share_directory(package_name)

    map_yaml_path = os.path.join(share_dir, 'maps', 'map.yaml')
    nav2_params_path = os.path.join(share_dir, 'config', 'nav2_params.yaml')
    bt_xml_path = os.path.join(share_dir, 'behavior_trees', 'replan_on_obstacle.xml')

    return LaunchDescription([

        # Static transform publisher for map -> odom (identity)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),

        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[map_yaml_path, {'use_sim_time': False}],
            output='screen'
        ),

        # Global costmap
        Node(
            package='nav2_costmap_2d',
            executable='costmap_2d_node',
            name='global_costmap',
            parameters=[nav2_params_path],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen'
        ),

        # Planner server with SmacPlannerHybrid
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_params_path],
            output='screen'
        ),

        # Your custom controller action server
        Node(
            package=package_name,
            executable='pure_pursuit_action_server',
            name='follow_path',
            parameters=[nav2_params_path],
            output='screen'
        ),

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_params_path, {'default_bt_xml_filename': bt_xml_path}],
            output='screen'
        ),

        # Lifecycle manager to auto-activate everything
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server', 'planner_server', 'bt_navigator', 'global_costmap']
            }],
            output='screen'
        ),
    ])
