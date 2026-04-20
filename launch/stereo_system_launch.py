from launch import LaunchDescription
from launch_ros.actions import Node

LEFT_CAMERA_ID = 1
RIGHT_CAMERA_ID = 0
JPEG_QUALITY = 50


def create_camera_node(side, camera_id):
    node_name = f"{side}_frame_publisher"
    camera_namespace = f"camera_{camera_id}"

    return Node(
        package="autonomous_navigation",
        executable="frame_publisher",
        name=node_name,
        parameters=[
            {"camera_id": camera_id},
            {
                f"{node_name}.{camera_namespace}.image.compressed.jpeg_quality": JPEG_QUALITY
            },
        ],
        remappings=[
            (
                f"/{node_name}/{camera_namespace}/image",
                f"/autonomous_vehicle/{side}_camera/image",
            ),
            (
                f"/{node_name}/{camera_namespace}/image/compressed",
                f"/{side}_camera/image/compressed",
            ),
        ],
    )


def generate_launch_description():
    return LaunchDescription(
        [
            create_camera_node("left", LEFT_CAMERA_ID),
            create_camera_node("right", RIGHT_CAMERA_ID),
        ]
    )
