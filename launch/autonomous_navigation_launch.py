from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share_dir = get_package_share_directory('autonomous_navigation')
    config_path = os.path.join(package_share_dir, 'config', 'ros_gz_bridge_config.yaml')
    model_path = os.path.join(package_share_dir, 'models', 'autonomous_vehicle')
    world_path = os.path.join(package_share_dir, 'worlds', 'autonomous_navigation_world.sdf')

    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument('world', default_value=world_path, description='Absolute path to a world file to open')

    ignition_gazebo_process = ExecuteProcess(
        cmd=['ign', 'gazebo', world], 
        output='screen'
    )
    return LaunchDescription([
        world_arg,
        ignition_gazebo_process,
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-file', model_path, '-z', '0.325']
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            ros_arguments=['-p', f'config_file:={config_path}']
        ),
        # Node(
        #     package='autonomous_navigation',
        #     executable='stereo_depth_estimator',
        #     name='stereo_depth_estimator'
        # )
    ])