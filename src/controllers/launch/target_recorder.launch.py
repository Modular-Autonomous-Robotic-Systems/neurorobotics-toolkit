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
    
    # Argument for the output bag file name (without extension).
    output_bag_name_arg = launch.actions.DeclareLaunchArgument(
        'output_bag_name',
        default_value='/ws/data/telemetry',
        description='Name of the output rosbag file.'
    )

    # Define the list of all supported topics.
    all_supported_topics_list = [
        "/ap/airspeed", "/ap/battery", "/ap/clock", "/ap/cmd_gps_pose", "/ap/cmd_vel",
        "/ap/geopose/filtered", "/ap/goal_lla", "/ap/gps_global_origin/filtered",
        "/ap/imu/experimental/data", "/ap/joy", "/ap/navsat", "/ap/pose/filtered",
        "/ap/status", "/ap/tf", "/ap/tf_static", "/ap/time", "/ap/twist/filtered"
    ]
    # Format the list into a string that looks like a Python list, e.g., "['/topic1', '/topic2']"
    # This is the format the ROS 2 parameter system expects for a vector of strings.
    default_topics_str = f"[{', '.join([f'{repr(topic)}' for topic in all_supported_topics_list])}]"

    topics_to_record_arg = launch.actions.DeclareLaunchArgument(
        'topics_to_record',
        default_value=default_topics_str,
        description="A string-formatted list of topics to record, e.g., \"['/topic1', '/topic2']\""
    )
 
    sensors_pkg_path = launch_ros.substitutions.FindPackageShare('sensors')
    # encoder_launch_file = launch.substitutions.PathJoinSubstitution([
    #     sensors_pkg_path,
    #     'launch',
    #     'ffmpeg_encode.launch.py'
    # ])
    # encoder_launch = launch.actions.IncludeLaunchDescription(
    #     launch.launch_description_sources.PythonLaunchDescriptionSource(encoder_launch_file),
    #     launch_arguments={
    #         'input_topic': '/airsim_node/Copter/front_center_Scene/image',
    #         'ffmpeg_topic': '/front_center_camera/compressed'}.items()
    # )
    # delayed_launch.actions.append(
    #     launch.actions.LogInfo(msg="Launching ffmpeg_encoder after ardupilot_sitl")
    # )
    # delayed_launch.actions.append(encoder_launch)
    
    logger_launch_file = launch.substitutions.PathJoinSubstitution([
        sensors_pkg_path,
        'launch',
        'managed_logger.launch.py'
    ])
    logger_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(logger_launch_file),
        launch_arguments={
            'topics_to_record': launch.substitutions.LaunchConfiguration('topics_to_record') ,
            'output_bag_name': launch.substitutions.LaunchConfiguration('output_bag_name')
        }.items()
    )
    
    video_logging_driver_node = launch_ros.actions.Node(
        package='controllers',
        executable='video_logging_controller_node',
        name='video_logging_controller',
        output='screen',
        respawn=False,
        arguments=['--ros-args', '--log-level', [log_level]],
        # prefix=['gdb -ex run -ex bt --args'],
        parameters=[{
            'lifecycle_node_to_manage': 'managed_logger_node', # Hardcoded target node name for the driver
            'ap_status_topic': ap_status_topic_name,   # Pass the status topic to the driver
        }],
    )

    return launch.LaunchDescription([
        log_level_arg,
        ap_status_topic_arg,
        output_bag_name_arg,
        topics_to_record_arg,
        logger_launch,
        video_logging_driver_node
    ])
