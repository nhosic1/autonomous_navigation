from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="autonomous_navigation",
                executable="frame_publisher",
                name="left_frame_publisher",
                parameters=[
                    {"camera_id": 0},
                    {"left_frame_publisher.camera_0.image.compressed.jpeg_quality": 50},
                ],
                remappings=[
                    ("/left_frame_publisher/camera_0/image", "/left_camera/image"),
                    (
                        "/left_frame_publisher/camera_0/image/compressed",
                        "/left_camera/image/compressed",
                    ),
                ],
            ),
            Node(
                package="autonomous_navigation",
                executable="frame_publisher",
                name="right_frame_publisher",
                parameters=[
                    {"camera_id": 1},
                    {
                        "right_frame_publisher.camera_1.image.compressed.jpeg_quality": 50
                    },
                ],
                remappings=[
                    ("/right_frame_publisher/camera_1/image", "/right_camera/image"),
                    (
                        "/right_frame_publisher/camera_1/image/compressed",
                        "/right_camera/image/compressed",
                    ),
                ],
            ),
        ]
    )
