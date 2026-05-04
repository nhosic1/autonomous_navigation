from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LEFT_CAMERA_ID = 1
RIGHT_CAMERA_ID = 0


def create_camera_node(side, camera_id, compressed_image_jpeg_quality):
    node_name = f"{side}_frame_publisher"
    camera_namespace = f"camera_{camera_id}"

    return Node(
        package="autonomous_navigation",
        executable="frame_publisher",
        name=node_name,
        parameters=[
            {"camera_id": camera_id},
            {
                f"{node_name}.{camera_namespace}.image.compressed.jpeg_quality": compressed_image_jpeg_quality
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
    compressed_image_jpeg_quality = LaunchConfiguration("compressed_image_jpeg_quality")
    compressed_image_jpeg_quality_arg = DeclareLaunchArgument(
        "compressed_image_jpeg_quality",
        default_value="50",
        description="JPEG quality for compressed image transport.",
    )

    return LaunchDescription(
        [
            compressed_image_jpeg_quality_arg,
            create_camera_node("left", LEFT_CAMERA_ID, compressed_image_jpeg_quality),
            create_camera_node("right", RIGHT_CAMERA_ID, compressed_image_jpeg_quality),
        ]
    )
