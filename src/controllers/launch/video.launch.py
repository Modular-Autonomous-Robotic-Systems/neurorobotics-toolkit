import launch
import launch_ros

def generate_launch_description():
    """
    Generates the launch description for starting the video_logger_test
    and its VideoLoggingDriver.
    """

    # Declare launch arguments for more flexibility
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

    # 1. Node action for the video_logger_node (the managed lifecycle node)
    video_logger_test_node = launch_ros.actions.Node(
        package='sensors',
        executable='video_logger_node', 
        name='video_logger_node', 
        output='screen',
        respawn=False, 
        arguments=['--ros-args', '--log-level', [log_level]], 
        parameters=[{
            'output_file_path': '/ws/data/output.mp4',
            'width' : 640,
            'height' : 480,
            'fps' : 30,
            'camera_topic' : '/airsim_node/Copter/front_center_Scene/image',
            'image_type' : 'bgr'
        }],
    )

    # 2. Node action for the VideoLoggingDriver
    video_logging_driver_node = launch_ros.actions.Node(
        package='controllers',
        executable='video_logging_controller_node',
        name='video_logging_controller',
        output='screen',
        respawn=False,
        arguments=['--ros-args', '--log-level', [log_level]],
        # prefix=['gdb -ex run -ex bt --args'],
        parameters=[{
            'lifecycle_node_to_manage': 'video_logger_node', # Hardcoded target node name for the driver
            'ap_status_topic': ap_status_topic_name,   # Pass the status topic to the driver
        }],
    )

    return launch.LaunchDescription([
        log_level_arg,
        ap_status_topic_arg,
        video_logger_test_node,
        video_logging_driver_node,
    ])
