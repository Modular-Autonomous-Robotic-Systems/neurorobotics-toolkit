import os

import launch_ros
from ament_index_python.packages import get_package_share_directory

import launch


def generate_launch_description():
    logger_default = "INFO"
    logger = launch.substitutions.LaunchConfiguration(
        "log-level",
        default=logger_default,
    )
    logger_arg = launch.actions.DeclareLaunchArgument(
        "log-level",
        default_value=[logger_default],
        description="Logging level for the nodes (DEBUG, INFO, WARN, ERROR, FATAL)",
    )
    ns_arg = launch.actions.DeclareLaunchArgument(
        "drone-name",
        default_value="drone1",
        description="Name of the Tello drone for ID",
    )
    ns = launch.substitutions.LaunchConfiguration("drone-name")
    camera_topic = (
        launch.substitutions.PythonExpression(["'/' + ", "'", ns, "' + '/image_raw'"]),
    )
    tello_status_topic = (
        launch.substitutions.PythonExpression(
            ["'/' + ", "'", ns, "' + '/tello_state'"]
        ),
    )
    controllers_pkg_path = get_package_share_directory("controllers")
    frontier_detection_path = get_package_share_directory("frontier_detection")
    sensors_pkg_path = get_package_share_directory("sensors")

    telemetry_path_arg = launch.actions.DeclareLaunchArgument(
        "telemetry_path",
        default_value="/ws/data/telemetry",
        description="Path to the ros2 bag to replay",
    )
    telemetry_path = launch.substitutions.LaunchConfiguration("telemetry_path")

    decoder_launch_file = launch.substitutions.PathJoinSubstitution(
        [sensors_pkg_path, "launch", "ffmpeg_decode.launch.py"]
    )
    decoder_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            decoder_launch_file
        ),
        launch_arguments={
            "compressed_input_topic": launch.substitutions.PythonExpression(
                ["'/' + ", "'", ns, "' + '/image_compressed'"]
            ),
            "raw_output_topic": launch.substitutions.PythonExpression(
                ["'/' + ", "'", ns, "' + '/image_raw'"]
            ),
        }.items(),
    )

    bag_play = launch.actions.ExecuteProcess(
        cmd=["ros2", "bag", "play", telemetry_path],
        output="screen",
    )

    delayed_bag_play = launch.actions.TimerAction(
        period=10.0,
        actions=[bag_play],
    )

    nodes = [
        logger_arg,
        ns_arg,
        telemetry_path_arg,
        decoder_launch,
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(controllers_pkg_path, "launch/slam_controller.launch.py")
            ),
            launch_arguments=[
                ("log-level", logger),
                ("camera-topic", camera_topic),
                (
                    "settings-file-path",
                    "/ws/ros_ws/src/slam/orb_slam3/config/Monocular/tello.yaml",
                ),
                (
                    "vocab-file-path",
                    "/ws/ros_ws/src/slam/orb_slam3/Vocabulary/ORBvoc.txt.bin",
                ),
                ("drone-type", "tello"),
                ("tello-status-topic-name", tello_status_topic),
            ],
        ),
        # OctoMap Builder Node
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    frontier_detection_path, "launch/frontier_detection_launch.py"
                )
            ),
            launch_arguments=[("log-level", logger)],
        ),
        delayed_bag_play,
    ]

    return launch.LaunchDescription(nodes)
