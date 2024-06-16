from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="autonomous_navigation",
                executable="frame_publisher",
                name="left_frame_publisher",
                ros_arguments=["-p", "camera_id:=0"],
                remappings=[
                    ("/left_frame_publisher/camera_0/image", "/left_camera/image")
                ],
            ),
            Node(
                package="autonomous_navigation",
                executable="frame_publisher",
                name="right_frame_publisher",
                ros_arguments=["-p", "camera_id:=1"],
                remappings=[
                    ("/right_frame_publisher/camera_1/image", "/right_camera/image")
                ],
            ),
        ]
    )
