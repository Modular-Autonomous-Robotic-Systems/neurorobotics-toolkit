import launch_ros

import launch


def generate_launch_description():
    log_level_arg = launch.actions.DeclareLaunchArgument(
        "log-level",
        default_value="INFO",
        description="Logging level for the nodes (DEBUG, INFO, WARN, ERROR, FATAL)",
    )
    ap_status_topic_arg = launch.actions.DeclareLaunchArgument(
        "ap_status_topic_name",
        default_value="/ap/status",  # Default AP status topic
        description="Topic name for ArduPilot status messages.",
    )
    ap_type_arg = launch.actions.DeclareLaunchArgument(
        "ap_type",
        default_value="ardupilot",
        description="Autopilot type to log data for supported options are `ardupilot` and `tello`",
    )
    output_bag_name_arg = launch.actions.DeclareLaunchArgument(
        "output_bag_name",
        default_value="/ws/data/telemetry",
        description="Name of the output rosbag file. The output bag name must not contain special symbols and numbers.",
    )
    airsim2_copter_params_arg = launch.actions.DeclareLaunchArgument(
        "ardupilot-params",
        default_value="/ws/sitl_ws/config/airsim2/project-airsim-quad.parm",
        description=(
            "Absolute path to the ArduPilot parameter file appended last to "
            "the SITL defaults list. Applied on top of copter.parm, "
            "dds_udp.parm and airsim-quadX.parm, so it need only carry the "
            "parameters that differ. The default is Project AirSim's own "
            "quadrotor tuning, copied from the upstream example scripts."
        ),
    )
    # Get LaunchConfiguration values to be used by nodes
    log_level = launch.substitutions.LaunchConfiguration("log-level")
    ap_status_topic_name = launch.substitutions.LaunchConfiguration(
        "ap_status_topic_name"
    )
    ap_type = launch.substitutions.LaunchConfiguration("ap_type")
    output_bag_name = launch.substitutions.LaunchConfiguration("output_bag_name")
    ardupilot_params = launch.substitutions.LaunchConfiguration("ardupilot-params")
    use_sim_time_arg = launch.actions.DeclareLaunchArgument(
        "use-sim-time",
        default_value="False",
        choices=["True", "False"],
        description=(
            "Boolean, the only accepted values are True and False. Run the SITL "
            "stack on Project AirSim's simulator clock. The bridge publishes "
            "the simulation clock on the global /clock unconditionally, so "
            "this flag decides only whether ArduPilot stamps every /ap/* "
            "message from it and whether the recorder runs on it. Default "
            "False reproduces the pre-2026-07-28 behaviour exactly."
        ),
    )
    use_sim_time = launch.substitutions.LaunchConfiguration("use-sim-time")

    delayed_launch = launch.actions.TimerAction(
        period=3.0,  # Seconds
        actions=[
            launch.actions.LogInfo(
                msg="Timer elapsed. Launching projectairsim_ros2_cpp node"
            ),
        ],
    )

    # --- Project AirSim Launch ---
    # projectairsim_ros2_cpp ships no launch file, so the node is declared
    # directly. Unlike Cosys-AirSim the simulator starts with no scene, so this
    # node is the primary client that loads one, which is why it carries
    # scene_config and sim_config_path.
    projectairsim_node = launch_ros.actions.Node(
        package="projectairsim_ros2_cpp",
        executable="projectairsim_ros2_cpp_node",
        name="projectairsim_ros2_cpp_node",
        output="screen",
        respawn=False,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                "address": "127.0.0.1",
                "port_topics": 8989,
                "port_services": 8990,
                "scene_config": "scene_ardu_copter.jsonc",
                "sim_config_path": "/ws/sitl_ws/config/airsim2",
                # Raised from the 2.0 default because StonePineForest is a
                # substantially heavier map than Blocks.
                "delay_after_load_sec": 5.0,
                # 200 Hz. Above the 3 ms simulation step and below the 6.17 ms
                # period of the fastest /ap/* publisher, so no two consecutive
                # ArduPilot samples can share a /clock stamp.
                "publish_clock_period_sec": 0.005,
                # The SLAM stack broadcasts its own map to camera transform.
                "publish_tf": False,
                "vehicle_name": "Copter",
            }
        ],
        remappings=[
            (
                "/ProjectAirsim/StonePineForestArduCopter/robots/Copter"
                "/sensors/front_center/scene_camera",
                "/airsim_node/Copter/front_center_Scene/image",
            ),
        ],
    )
    delayed_launch.actions.append(projectairsim_node)

    # --- ArduPilot SITL Launch ---
    # Assuming workspace overlay of /ardu_ws
    ardupilot_sitl_pkg_path = launch_ros.substitutions.FindPackageShare(
        "ardupilot_sitl"
    )
    ardupilot_launch_file = launch.substitutions.PathJoinSubstitution(
        [ardupilot_sitl_pkg_path, "launch", "sitl_dds_udp.launch.py"]
    )

    # Construct the full path for the parameter files
    default_copter_params = launch.substitutions.PathJoinSubstitution(
        [ardupilot_sitl_pkg_path, "config", "default_params", "copter.parm"]
    )

    default_airsim_copter_params = launch.substitutions.PathJoinSubstitution(
        [ardupilot_sitl_pkg_path, "config", "default_params", "airsim-quadX.parm"]
    )

    default_dds_udp_params = launch.substitutions.PathJoinSubstitution(
        [ardupilot_sitl_pkg_path, "config", "default_params", "dds_udp.parm"]
    )

    ardupilot_sitl_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            ardupilot_launch_file
        ),
        launch_arguments={
            "transport": "udp4",
            "synthetic_clock": "True",
            "use_sim_time": use_sim_time,
            "wipe": "False",
            "model": "airsim-copter",
            "speedup": "1",
            "slave": "0",
            "instance": "0",
            # The Project AirSim tuning is appended rather than substituted.
            # Its own header specifies --add-param-file, and airsim-copter is
            # defined by vehicleinfo.json as copter.parm plus airsim-quadX.parm,
            # so substituting would silently drop FRAME_TYPE, SCHED_LOOP_RATE,
            # MOT_THST_HOVER, the GPS lags and every filter.
            "defaults": [
                default_copter_params,
                ",",
                default_dds_udp_params,
                ",",
                default_airsim_copter_params,
                ",",
                ardupilot_params,
            ],  # Note: passing list for multiple files
            "sim_address": "127.0.0.1",  # This was duplicated in your command, ensuring one is passed
            "master": "tcp:127.0.0.1:5760",
            "sitl": "127.0.0.1:5501",
            "map": "False",
            "console": "False",
            "sim_port_in": "9003",
            "sim_port_out": "9002",
            # 'sim_address': '127.0.0.1' # Already included
        }.items(),
    )
    delayed_launch.actions.append(
        launch.actions.LogInfo(msg="Launching ardupilot_sitl after projectairsim.")
    )
    delayed_launch.actions.append(ardupilot_sitl_launch)

    sensors_pkg_path = launch_ros.substitutions.FindPackageShare("sensors")
    encoder_launch_file = launch.substitutions.PathJoinSubstitution(
        [sensors_pkg_path, "launch", "ffmpeg_encode.launch.py"]
    )
    encoder_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            encoder_launch_file
        ),
        launch_arguments={
            "input_topic": "/airsim_node/Copter/front_center_Scene/image",
            "ffmpeg_topic": "/front_center_camera/compressed",
        }.items(),
    )
    delayed_launch.actions.append(
        launch.actions.LogInfo(msg="Launching ffmpeg_encoder after ardupilot_sitl")
    )
    delayed_launch.actions.append(encoder_launch)
    controllers_pkg_path = launch_ros.substitutions.FindPackageShare("controllers")

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
        "/front_center_camera/compressed",
    ]
    topics_str = (
        f"[{', '.join([f'{repr(topic)}' for topic in all_supported_topics_list])}]"
    )
    topics_to_record_arg = launch.actions.DeclareLaunchArgument(
        "topics_to_record",
        default_value=topics_str,
        description="A string-formatted list of topics to record, e.g., \"['/topic1', '/topic2']\"",
    )
    topics_to_record = launch.substitutions.LaunchConfiguration("topics_to_record")
    managed_logger_launch_file = launch.substitutions.PathJoinSubstitution(
        [controllers_pkg_path, "launch", "target_recorder.launch.py"]
    )
    managed_logger_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            managed_logger_launch_file
        ),
        launch_arguments={
            "log-level": log_level,
            "ap_status_topic_name": ap_status_topic_name,
            "ap_type": ap_type,
            "topics_to_record": topics_to_record,
            "output_bag_name": output_bag_name,
            "use-sim-time": use_sim_time,
        }.items(),
    )
    delayed_launch.actions.append(
        launch.actions.LogInfo(msg="Launching logger after ardupilot_sitl encoder")
    )
    delayed_launch.actions.append(managed_logger_launch)

    return launch.LaunchDescription(
        [
            log_level_arg,
            ap_status_topic_arg,
            ap_type_arg,
            output_bag_name_arg,
            airsim2_copter_params_arg,
            topics_to_record_arg,
            use_sim_time_arg,
            launch.actions.LogInfo(
                msg="Assuming Project AirSim simulator is starting/started externally."
            ),
            launch.actions.LogInfo(
                msg="Introducing a 3-second delay before launching projectairsim_ros2_cpp."
            ),
            delayed_launch,
        ]
    )
