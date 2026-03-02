import os

import launch_ros
from ament_index_python.packages import get_package_share_directory

import launch


def generate_launch_description():
    logger_default = "INFO"
    logger = launch.substitutions.LaunchConfiguration(
        "log-level", default=logger_default
    )
    logger_arg = launch.actions.DeclareLaunchArgument(
        "log-level", default_value=[logger_default], description="Logging level"
    )

    camera_topic_default = "/camera"
    camera_topic = launch.substitutions.LaunchConfiguration(
        "camera-topic", default=camera_topic_default
    )
    camera_topic_arg = launch.actions.DeclareLaunchArgument(
        "camera-topic",
        default_value=[camera_topic_default],
        description="Camera Topic name to listen to for images",
    )

    settings_file_path_default = (
        "/ws/ros_ws/src/slam/orb_slam3/config/Monocular/sitl.yaml"
    )
    settings_file_path = launch.substitutions.LaunchConfiguration(
        "settings-file-path", default=settings_file_path_default
    )
    settings_file_path_arg = launch.actions.DeclareLaunchArgument(
        "settings-file-path",
        default_value=[settings_file_path_default],
        description="Path to ORBSLAM3 settings file",
    )

    vocab_file_path_default = "/ws/ros_ws/src/slam/orb_slam3/Vocabulary/ORBvoc.txt.bin"
    vocab_file_path = launch.substitutions.LaunchConfiguration(
        "vocab-file-path", default=vocab_file_path_default
    )
    vocab_file_path_arg = launch.actions.DeclareLaunchArgument(
        "vocab-file-path",
        default_value=[vocab_file_path_default],
        description="Path to ORBSLAM3 vocabulary file",
    )

    drone_type_default = "ardupilot"
    drone_type = launch.substitutions.LaunchConfiguration(
        "drone-type", default=drone_type_default
    )
    drone_type_arg = launch.actions.DeclareLaunchArgument(
        "drone-type",
        default_value=[drone_type_default],
        description="Type of drone platform to start SLAM for. Can take one of: ardupilot or tello",
    )

    ardupilot_status_topic_name_default = "/ap/status"
    ardupilot_status_topic_name = launch.substitutions.LaunchConfiguration(
        "arduilot-status-topic-name", default=ardupilot_status_topic_name_default
    )
    ardupilot_status_topic_name_arg = launch.actions.DeclareLaunchArgument(
        "arduilot-status-topic-name",
        default_value=[ardupilot_status_topic_name_default],
        description="Topic Name for SLAM controller to listen to for ArduPilot Status",
    )

    tello_status_topic_name_default = "/tello_state"
    tello_status_topic_name = launch.substitutions.LaunchConfiguration(
        "tello-status-topic-name", default=tello_status_topic_name_default
    )
    tello_status_topic_name_arg = launch.actions.DeclareLaunchArgument(
        "tello-status-topic-name",
        default_value=[tello_status_topic_name_default],
        description="Topic Name for SLAM controller to listen to for tello Status",
    )

    delayed_launch = launch.actions.TimerAction(
        period=3.0,  # Seconds
        actions=[
            launch_ros.actions.Node(
                package="slam",
                executable="orbslam3_mono_node",
                name="orbslam3_mono_node",
                respawn=False,
                prefix=["gdb -ex run -ex bt --args"],
                arguments=["--ros-args", "--log-level", logger],
                parameters=[
                    {
                        "settings_file_path": settings_file_path,
                        "camera_topic_name": camera_topic,
                        "vocab_file_path": vocab_file_path,
                    }
                ],
                output="screen",
            ),
            launch_ros.actions.Node(
                package="controllers",
                executable="orbslam3_monocular_controller",
                output="screen",
                namespace="/",
                name="orbslam3_controller",
                respawn=False,
                arguments=["--ros-args", "--log-level", logger],
                parameters=[
                    {
                        "settings_file_path": settings_file_path,
                        "camera_topic_name": camera_topic,
                        "vocab_file_path": vocab_file_path,
                        "drone_type": drone_type,
                        "ardupilot_status_topic": ardupilot_status_topic_name,
                        "tello_status_topic": tello_status_topic_name,
                    }
                ],
            ),
        ],
    )

    nodes = [
        logger_arg,
        camera_topic_arg,
        settings_file_path_arg,
        vocab_file_path_arg,
        drone_type_arg,
        ardupilot_status_topic_name_arg,
        tello_status_topic_name_arg,
        delayed_launch,
    ]

    return launch.LaunchDescription(nodes)
