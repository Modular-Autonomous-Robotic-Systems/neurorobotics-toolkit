import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
    input_topic_arg = DeclareLaunchArgument(
            'input_topic',
            default_value='/camera/image_raw',
            description='The input topic for the raw image stream.'
            )

    ffmpeg_topic_arg = DeclareLaunchArgument(
            'ffmpeg_topic',
            default_value='/camera/image_ffmpeg',
            description='The topic for the ffmpeg compressed stream.'
            )

    # --- Get Launch Configurations ---
    input_topic = LaunchConfiguration('input_topic')
    ffmpeg_topic = LaunchConfiguration('ffmpeg_topic')

    # --- Nodes ---

    # 1. Republish node to compress the raw image stream into ffmpeg format.
    raw_to_ffmpeg_node = Node(
            package='image_transport',
            executable='republish',
            name='raw_to_ffmpeg_republisher',
            arguments=[
                'raw',          # Input transport
                'ffmpeg'        # Output transport
                ],
            remappings=[
                ('in', input_topic),
                ('out/ffmpeg', ffmpeg_topic)
                ],
            parameters=[{
                # You can customize FFmpeg encoding parameters here.
                # Find available encoders with: ffmpeg -codecs
                # Find available presets with: ffmpeg -hide_banner -f lavfi -i nullsrc -c:v libx264 -preset help -f mp4 - 2>&1
                'ffmpeg_image_transport.encoding': 'libx264',
                'ffmpeg_image_transport.profile': 'main',
                'ffmpeg_image_transport.preset': 'ultrafast', # Prioritize speed
                'ffmpeg_image_transport.tune': 'zerolatency', # Good for live streaming
                'ffmpeg_image_transport.gop_size': 15,         # Group of pictures size
                'ffmpeg_image_transport.bit_rate': 1000000,    # In bits/s
                }],
            output='screen'
            )

    return LaunchDescription([
        input_topic_arg,
        ffmpeg_topic_arg,

        LogInfo(msg=["Subscribing to raw images on topic: ", input_topic]),
        LogInfo(msg=["Compressing to ffmpeg on topic: ", ffmpeg_topic]),

        raw_to_ffmpeg_node,
    ])
