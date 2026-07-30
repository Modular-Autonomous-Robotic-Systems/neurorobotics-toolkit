import os

import ament_index_python
import launch
import launch_ros


def generate_launch_description():

    slam_launch_dir = os.path.join(
        ament_index_python.packages.get_package_share_directory("slam"), "launch"
    )

    logger_default = "INFO"
    log_level = launch.substitutions.LaunchConfiguration(
        "log-level", default=logger_default
    )
    log_level_arg = launch.actions.DeclareLaunchArgument(
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

    imu_topic_default = ""
    imu_topic = launch.substitutions.LaunchConfiguration(
        "imu-topic", default=imu_topic_default
    )
    imu_topic_arg = launch.actions.DeclareLaunchArgument(
        "imu-topic",
        default_value=[imu_topic_default],
        description="IMU Topic name (required for VISLAM)",
    )

    calibration_file_path_default = ""
    calibration_file_path = launch.substitutions.LaunchConfiguration(
        "calibration-file-path", default=calibration_file_path_default
    )
    calibration_file_path_arg = launch.actions.DeclareLaunchArgument(
        "calibration-file-path",
        default_value=[calibration_file_path_default],
        description="Path to Basalt Calibration File",
    )

    configuration_file_path_default = ""
    configuration_file_path = launch.substitutions.LaunchConfiguration(
        "configuration-file-path", default=configuration_file_path_default
    )
    configuration_file_path_arg = launch.actions.DeclareLaunchArgument(
        "configuration-file-path",
        default_value=[configuration_file_path_default],
        description="Path to Basalt Configuration File",
    )
    use_sim_time_arg = launch.actions.DeclareLaunchArgument(
        "use-sim-time",
        default_value="False",
        choices=["True", "False"],
        description="Boolean, the only accepted values are True and False. Run the SLAM compute node on the simulator clock published on /clock, so that the stamps it produces share a domain with the camera and inertial stamps it consumes.",
    )
    use_sim_time = launch.substitutions.LaunchConfiguration("use-sim-time")

    use_visualisation_arg = launch.actions.DeclareLaunchArgument(
        "use-visualisation",
        default_value="False",
        choices=["True", "False"],
        description="Boolean, the only accepted values are True and False. Forwarded to basalt_slam.launch.py, where it opens the Basalt Pangolin window and renders the live VIO trajectory, image grid, sliding-window landmarks and local map. Requires a reachable X display, so leave it False for headless runs.",
    )
    use_visualisation = launch.substitutions.LaunchConfiguration("use-visualisation")

    slam_type_default = "VSLAM"
    slam_type = launch.substitutions.LaunchConfiguration(
        "slam-type", default=slam_type_default
    )
    slam_type_arg = launch.actions.DeclareLaunchArgument(
        "slam-type",
        default_value=[slam_type_default],
        description="Type of SLAM to configure and activate. This node supports VSLAM and VISLAM",
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

    ap_status_topic_name_default = "/ap/status"
    ap_status_topic_name = launch.substitutions.LaunchConfiguration(
        "ap_status_topic_name", default=ap_status_topic_name_default
    )
    ap_status_topic_name_arg = launch.actions.DeclareLaunchArgument(
        "ap_status_topic_name",
        default_value=[ap_status_topic_name_default],
        description="Topic Name for SLAM controller to listen to for ArduPilot Status",
    )

    basalt_slam_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(slam_launch_dir, "basalt_slam.launch.py")
        ),
        launch_arguments={
            "log-level": log_level,
            "camera-topic": camera_topic,
            "calibration-file-path": calibration_file_path,
            "configuration-file-path": configuration_file_path,
            "slam-type": slam_type,
            "imu-topic": imu_topic,
            "use-sim-time": use_sim_time,
            "use-visualisation": use_visualisation,
        }.items(),
    )

    basalt_controller_node = launch_ros.actions.Node(
        package="controllers",
        executable="basalt_slam_controller",
        name="basalt_controller",
        namespace="/",
        output="screen",
        respawn=False,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                "slam_compute_node_name": "basalt_slam_node",
                "drone_type": drone_type,
                "ardupilot_status_topic": ap_status_topic_name,
                "camera_topic_name": camera_topic,
                "imu_topic_name": imu_topic,
                "config_file_path": configuration_file_path,
                "calib_file_path": calibration_file_path,
            }
        ],
    )

    delayed_seq_launch = launch.actions.TimerAction(
        period=0.0, actions=[basalt_slam_launch]
    )

    delayed_controller_launch = launch.actions.TimerAction(
        period=3.0, actions=[basalt_controller_node]
    )

    nodes = [
        log_level_arg,
        camera_topic_arg,
        imu_topic_arg,
        calibration_file_path_arg,
        configuration_file_path_arg,
        slam_type_arg,
        drone_type_arg,
        ap_status_topic_name_arg,
        use_sim_time_arg,
        use_visualisation_arg,
        delayed_seq_launch,
        delayed_controller_launch,
    ]

    return launch.LaunchDescription(nodes)
