# ffmpeg_republish_launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for testing ffmpeg_image_transport.

    This launch file starts two image_transport republish nodes:
    1.  raw_to_ffmpeg: Subscribes to a raw image topic, compresses it using
        the ffmpeg plugin, and republishes it on a compressed topic.
    2.  ffmpeg_to_raw: Subscribes to the compressed topic, decompresses it,
        and republishes it as a raw image topic suitable for visualization.

    It also includes a commented-out node to launch RViz for visualization.
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

    output_topic_arg = DeclareLaunchArgument(
            'output_topic',
            default_value='/camera/image_republished',
            description='The output topic for the republished raw image stream.'
            )

    # --- Get Launch Configurations ---
    input_topic = LaunchConfiguration('input_topic')
    ffmpeg_topic = LaunchConfiguration('ffmpeg_topic')
    output_topic = LaunchConfiguration('output_topic')

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

    # 2. Republish node to decompress the ffmpeg stream back to a raw image.
    ffmpeg_to_raw_node = Node(
            package='image_transport',
            executable='republish',
            name='ffmpeg_to_raw_republisher',
            arguments=[
                'ffmpeg',       # Input transport
                'raw'           # Output transport
                ],
            remappings=[
                ('in/ffmpeg', ffmpeg_topic),
                ('out', output_topic)
                ],
            output='screen'
            )

    # 3. (Optional) RViz2 node for visualization.
    # To use this, you will need a rviz_config file.
    # You can create one by running `rviz2`, configuring the display,
    # and saving the configuration (File -> Save Config As).
    # rviz_config_file = os.path.join(
    #     get_package_share_directory('your_package_name'),
    #     'rviz',
    #     'ffmpeg_test.rviz'
    # )
    #
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     output='screen'
    # )

    return LaunchDescription([
        input_topic_arg,
        ffmpeg_topic_arg,
        output_topic_arg,

        LogInfo(msg=["Publishing raw images to topic: ", input_topic]),
        LogInfo(msg=["Compressing to ffmpeg on topic: ", ffmpeg_topic]),
        LogInfo(msg=["Decompressing to raw on topic: ", output_topic]),

        raw_to_ffmpeg_node,
        ffmpeg_to_raw_node,
        # rviz_node # Uncomment to launch RViz2
    ])
