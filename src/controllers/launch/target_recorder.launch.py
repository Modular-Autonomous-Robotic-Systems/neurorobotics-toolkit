import os

import launch
import launch_ros


def generate_launch_description():
    log_level_arg = launch.actions.DeclareLaunchArgument(
        "log-level",
        default_value="INFO",
        description="Logging level for the nodes (DEBUG, INFO, WARN, ERROR, FATAL)",
    )
    ap_status_topic_arg = launch.actions.DeclareLaunchArgument(
        "ap_status_topic_name",
        default_value="/ap/status",
        description="Topic name for Auto Pilot status messages.",
    )
    ap_type_arg = launch.actions.DeclareLaunchArgument(
        "ap_type",
        default_value="ardupilot",
        description="Autopilot type to log data for supported options are `ardupilot` and `tello`",
    )
    # Get LaunchConfiguration values to be used by nodes
    log_level = launch.substitutions.LaunchConfiguration("log-level")
    ap_status_topic_name = launch.substitutions.LaunchConfiguration(
        "ap_status_topic_name"
    )
    ap_type = launch.substitutions.LaunchConfiguration("ap_type")

    # Argument for the output bag file name (without extension).
    output_bag_name_arg = launch.actions.DeclareLaunchArgument(
        "output_bag_name",
        default_value="/ws/data/telemetry",
        description="Name of the output rosbag file. The output bag name must not contain special symbols and numbers.",
    )
    use_sim_time_arg = launch.actions.DeclareLaunchArgument(
        "use-sim-time",
        default_value="False",
        choices=["True", "False"],
        description="Boolean, the only accepted values are True and False. Run the managed recorder on the simulator clock published on /clock.",
    )
    use_sim_time = launch.substitutions.LaunchConfiguration("use-sim-time")

    # Define the list of all supported topics.
    all_supported_topics_list = [
        "/ap/airspeed",
        "/ap/battery",
        "/ap/clock",
        "/ap/cmd_gps_pose",
        "/ap/cmd_vel",
        "/ap/geopose/filtered",
        "/ap/goal_lla",
        "/ap/gps_global_origin/filtered",
        "/ap/imu/experimental/data",
        "/ap/joy",
        "/ap/navsat",
        "/ap/pose/filtered",
        "/ap/status",
        "/ap/tf",
        "/ap/tf_static",
        "/ap/time",
        "/ap/twist/filtered",
        "/tf",
        "/tf_static",
    ]
    # Format the list into a string that looks like a Python list, e.g., "['/topic1', '/topic2']"
    # This is the format the ROS 2 parameter system expects for a vector of strings.
    default_topics_str = (
        f"[{', '.join([f'{repr(topic)}' for topic in all_supported_topics_list])}]"
    )

    topics_to_record_arg = launch.actions.DeclareLaunchArgument(
        "topics_to_record",
        default_value=default_topics_str,
        description="A string-formatted list of topics to record, e.g., \"['/topic1', '/topic2']\"",
    )

    sensors_pkg_path = launch_ros.substitutions.FindPackageShare("sensors")

    logger_launch_file = launch.substitutions.PathJoinSubstitution(
        [sensors_pkg_path, "launch", "managed_logger.launch.py"]
    )
    logger_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            logger_launch_file
        ),
        launch_arguments={
            "topics_to_record": launch.substitutions.LaunchConfiguration(
                "topics_to_record"
            ),
            "output_bag_name": launch.substitutions.LaunchConfiguration(
                "output_bag_name"
            ),
            "use-sim-time": use_sim_time,
        }.items(),
    )

    video_logging_driver_node = launch_ros.actions.Node(
        package="controllers",
        executable="video_logging_controller_node",
        name="video_logging_controller",
        output="screen",
        respawn=False,
        arguments=["--ros-args", "--log-level", [log_level]],
        # prefix=['gdb -ex run -ex bt --args'],
        parameters=[
            {
                "lifecycle_node_to_manage": "managed_logger_node",  # Hardcoded target node name for the driver
                "ap_status_topic": ap_status_topic_name,  # Pass the status topic to the driver
                "ap_type": ap_type,
                "start_topic": "/video_logging_controller_node/start_logging",
            }
        ],
    )

    return launch.LaunchDescription(
        [
            log_level_arg,
            ap_status_topic_arg,
            ap_type_arg,
            output_bag_name_arg,
            topics_to_record_arg,
            use_sim_time_arg,
            logger_launch,
            video_logging_driver_node,
        ]
    )
