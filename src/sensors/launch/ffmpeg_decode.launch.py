import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generates the launch description for deploying ffmpeg_image_transport for image compression.

    This launch file starts two image_transport republish nodes:
    1.  raw_to_ffmpeg: Subscribes to a raw image topic, compresses it using
        the ffmpeg plugin, and republishes it on a compressed topic.

    """

    # --- Declare Launch Arguments ---
    # You can change these default values when you launch the file.
    # Example: ros2 launch your_package_name ffmpeg_republish_launch.py input_topic:=/my_camera/image_raw
    compressed_topic_arg = DeclareLaunchArgument(
        "compressed_input_topic",
        default_value="/camera/image_compressed",
        description="The input topic for the compressed image stream to be decoded.",
    )

    raw_output_topic_arg = DeclareLaunchArgument(
        "raw_output_topic",
        default_value="/camera/image_raw",
        description="The topic for the output raw iamge stream.",
    )

    node_name_arg = DeclareLaunchArgument(
        "ffmpeg_node_name",
        default_value="ffmpeg_to_raw_republisher",
        description="The name of the ffmpeg decoding node",
    )

    # --- Get Launch Configurations ---
    compressed_topic = LaunchConfiguration("compressed_input_topic")
    raw_output_topic = LaunchConfiguration("raw_output_topic")
    node_name = LaunchConfiguration("ffmpeg_node_name")

    # --- Nodes ---

    # 1. Republish node to compress the raw image stream into ffmpeg format.
    ffmpeg_to_raw_node = Node(
        package="image_transport",
        executable="republish",
        name=node_name,
        arguments=["ffmpeg", "raw"],  # Input transport  # Output transport
        remappings=[("in/ffmpeg", compressed_topic), ("out", raw_output_topic)],
        parameters=[
            {
                "in.ffmpeg.decoders.h264": "h264",
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            compressed_topic_arg,
            raw_output_topic_arg,
            node_name_arg,
            LogInfo(msg=["Subscribing to raw images on topic: ", compressed_topic]),
            LogInfo(msg=["Compressing to ffmpeg on topic: ", raw_output_topic]),
            ffmpeg_to_raw_node,
        ]
    )
