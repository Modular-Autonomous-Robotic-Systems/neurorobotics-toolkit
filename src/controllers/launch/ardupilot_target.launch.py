import launch
import launch_ros

def generate_launch_description():
    """
    Generates a launch description that wraps the ArduPilot SITL micro-ROS agent launch file.
    """
    
    verbosity_arg = launch.actions.DeclareLaunchArgument(
        'verbosity',
        default_value='4',
        description='Set the verbosity level for logging (0-6).'
    )

    device_arg = launch.actions.DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyS0',
        description='The device path for the serial connection.'
    )

    baud_rate_arg = launch.actions.DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='The baud rate for the serial connection.'
    )
    
    ap_status_topic_arg = launch.actions.DeclareLaunchArgument(
        "ap_status_topic_name",
        default_value="/ap/status", # Default AP status topic
        description="Topic name for ArduPilot status messages."
    )
    
    declared_args = [
        verbosity_arg,
        device_arg,
        baud_rate_arg,
        ap_status_topic_arg
    ]

    # Assuming /ardu_ws is sourced before this launch file is executed
    ardupilot_agent_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource([
            launch_ros.substitutions.FindPackageShare('ardupilot_sitl'),
            '/launch/micro_ros_agent.launch.py'
        ]),
        launch_arguments={
            'transport': 'serial',
            'verbosity': launch.substitutions.LaunchConfiguration('verbosity'),
            'device': launch.substitutions.LaunchConfiguration('device'),
            'baudrate': launch.substitutions.LaunchConfiguration('baud_rate')
        }.items()
    )
    
    sensors_pkg_path = launch_ros.substitutions.FindPackageShare('sensors')
    encoder_launch_file = launch.substitutions.PathJoinSubstitution([
        sensors_pkg_path,
        'launch',
        'ffmpeg_encode.launch.py'
    ])
    encoder_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(encoder_launch_file),
        launch_arguments={
            'input_topic': '/camera',
            'ffmpeg_topic': '/camera/compressed'}.items()
    )
    controllers_pkg_path = launch_ros.substitutions.FindPackageShare('controllers')
    all_supported_topics_list = [
        "/ap/airspeed", "/ap/battery", "/ap/clock", "/ap/cmd_gps_pose", "/ap/cmd_vel",
        "/ap/geopose/filtered", "/ap/goal_lla", "/ap/gps_global_origin/filtered",
        "/ap/imu/experimental/data", "/ap/joy", "/ap/navsat", "/ap/pose/filtered",
        "/ap/status", "/ap/tf", "/ap/tf_static", "/ap/time", "/ap/twist/filtered",
        "/camera/compressed"
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
            'ap_status_topic_name': launch.substitutions.LaunchConfiguration('ap_status_topic_name'),
            'topics_to_record': topics_str,
            'output_bag_name': '/ws/data/telemetry'}.items()
    )
    

    launch_entities = declared_args + [ardupilot_agent_launch] + [encoder_launch, ]

    return launch.LaunchDescription(launch_entities)


