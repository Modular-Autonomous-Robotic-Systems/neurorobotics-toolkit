import os
import launch
import launch_ros

def generate_launch_description():
    log_level_arg = launch.actions.DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        description="Logging level for the nodes (DEBUG, INFO, WARN, ERROR, FATAL)"
    )
    ap_status_topic_arg = launch.actions.DeclareLaunchArgument(
        "ap_status_topic_name",
        default_value="/ap/status", # Default AP status topic
        description="Topic name for ArduPilot status messages."
    )
    # Get LaunchConfiguration values to be used by nodes
    log_level = launch.substitutions.LaunchConfiguration("log_level")
    ap_status_topic_name = launch.substitutions.LaunchConfiguration("ap_status_topic_name")

    delayed_launch = launch.actions.TimerAction(
        period=3.0,  # Seconds
        actions=[
            launch.actions.LogInfo(msg="Timer elapsed. Launching airsim_ros_pkgs node"),
            # Action to launch airsim_ros_pkgs (defined below)
            # This will be added to the actions list of TimerAction
        ]
    )
    # --- AirSim Launch ---
    # Assuming workspace overlay of /airsim_ws
    airsim_ros_pkg_path = launch_ros.substitutions.FindPackageShare('airsim_ros_pkgs')
    airsim_launch_file = launch.substitutions.PathJoinSubstitution([
        airsim_ros_pkg_path,
        'launch',
        'airsim_node.launch.py'
    ])

    airsim_node_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(airsim_launch_file),
        launch_arguments={'host_ip': '127.0.0.1'}.items()
    )
    delayed_launch.actions.append(airsim_node_launch)

    # --- ArduPilot SITL Launch ---
    # Assuming workspace overlay of /ardupilot_ws
    ardupilot_sitl_pkg_path = launch_ros.substitutions.FindPackageShare('ardupilot_sitl')
    ardupilot_launch_file = launch.substitutions.PathJoinSubstitution([
        ardupilot_sitl_pkg_path,
        'launch',
        'sitl_dds_udp.launch.py'
    ])

    # Construct the full path for the parameter files
    default_copter_params = launch.substitutions.PathJoinSubstitution([
        ardupilot_sitl_pkg_path,
        'config',
        'default_params',
        'copter.parm'
    ])
    
    default_airsim_copter_params = launch.substitutions.PathJoinSubstitution([
        ardupilot_sitl_pkg_path,
        'config',
        'default_params',
        'airsim-quadX.parm'
    ])

    default_dds_udp_params = launch.substitutions.PathJoinSubstitution([
        ardupilot_sitl_pkg_path,
        'config',
        'default_params',
        'dds_udp.parm'
    ])

    ardupilot_sitl_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(ardupilot_launch_file),
        launch_arguments={
            'transport': 'udp4',
            'synthetic_clock': 'True',
            'wipe': 'False',
            'model': 'airsim-copter',
            'speedup': '1',
            'slave': '0',
            'instance': '0',
            'defaults': [default_copter_params, ',', default_dds_udp_params, ',', default_airsim_copter_params], # Note: passing list for multiple files
            'sim_address': '127.0.0.1', # This was duplicated in your command, ensuring one is passed
            'master': 'tcp:127.0.0.1:5760',
            'sitl': '127.0.0.1:5501',
            'map': 'False',
            'console': 'False',
            'sim_port_in': '9003',
            'sim_port_out': '9002'
            # 'sim_address': '127.0.0.1' # Already included
        }.items()
    )
    delayed_launch.actions.append(
        launch.actions.LogInfo(msg="Launching ardupilot_sitl after airsim_ros_pkgs.")
    )
    delayed_launch.actions.append(ardupilot_sitl_launch)
    
    sensors_pkg_path = launch_ros.substitutions.FindPackageShare('sensors')
    controllers_pkg_path = launch_ros.substitutions.FindPackageShare('controllers')
    
    all_supported_topics_list = [
        "/ap/airspeed", "/ap/battery", "/ap/clock", "/ap/cmd_gps_pose", "/ap/cmd_vel",
        "/ap/geopose/filtered", "/ap/goal_lla", "/ap/gps_global_origin/filtered",
        "/ap/imu/experimental/data", "/ap/joy", "/ap/navsat", "/ap/pose/filtered",
        "/ap/status", "/ap/tf", "/ap/tf_static", "/ap/time", "/ap/twist/filtered",
        "/airsim_node/Copter/front_center_Scene/image/ffmpeg"
    ]
    topics_str = f"[{', '.join([f'{repr(topic)}' for topic in all_supported_topics_list])}]"
    managed_logger_launch_file = launch.substitutions.PathJoinSubstitution([
        controllers_pkg_path,
        'launch',
        'target_recorder.launch.py'
    ])
    managed_logger_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(managed_logger_launch_file),
        launch_arguments={
            'log_level': log_level,
            'ap_status_topic_name': ap_status_topic_name,
            'topics_to_record': topics_str,
            'output_bag_name': '/ws/data/telemetry'}.items()
    )
    delayed_launch.actions.append(
        launch.actions.LogInfo(msg="Launching logger after ardupilot_sitl encoder")
    )
    delayed_launch.actions.append(managed_logger_launch)

    return launch.LaunchDescription([
        log_level_arg,
        ap_status_topic_arg,
        launch.actions.LogInfo(msg="Assuming AirSim simulator is starting/started externally."),
        launch.actions.LogInfo(msg="Introducing a 3-second delay before launching airsim_ros_pkgs."),
        delayed_launch
    ])
