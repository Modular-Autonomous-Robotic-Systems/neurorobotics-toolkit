import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    logger_default = "INFO"
    logger = launch.substitutions.LaunchConfiguration("log-level", default = logger_default)
    logger_arg = launch.actions.DeclareLaunchArgument(
            "log-level",
            default_value=[logger_default],
            description="Logging level")
    frontier_detection_path = get_package_share_directory('frontier_detection')
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
    all_supported_topics_list = [
        '/camera/compressed'
    ]
    topics_str = f"[{', '.join([f'{repr(topic)}' for topic in all_supported_topics_list])}]"
    logger_launch_file = launch.substitutions.PathJoinSubstitution([
        sensors_pkg_path,
        'launch',
        'managed_logger.launch.py'
    ])
    logger_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(logger_launch_file),
        launch_arguments={
            'topics_to_record': topics_str,
            'output_bag_name': '/ws/data/telemetry'}.items()
    )
    nodes = [
        # Tello driver node
        launch_ros.actions.Node(
            package='tello_driver',
            executable='tello_driver_main',
            output='screen',
            namespace='/',
            arguments=['--ros-args', '--log-level', logger],
            name='tello',
            remappings=[
                ('/image_raw', '/camera')
            ],
            respawn=True
        ),        
        encoder_launch,
        logger_launch,
        # ORB SLAM 3 Controller
        # TODO set parameters for node through substitutions
        launch_ros.actions.Node(
            package='controllers',
            executable='orbslam3_monocular_controller',
            output='screen',
            namespace='/',
            name='orbslam3_controller',
            respawn=True,
            arguments=['--ros-args', '--log-level', logger],
            parameters = [{
                'settings_path': '/ws/ros_ws/src/slam/config/orbslam3_mono_config.yaml',
                'camera_topic': '/camera',
            }]
        ),

        # ORB SLAM3 compute node
        launch_ros.actions.Node(
            package='slam',
            executable='orbslam3_mono_node',
            name='orbslam3_mono_node',
            respawn=True,
            arguments=['--ros-args', '--log-level', logger],
            output='screen'
        ),

        # OctoMap Builder Node
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(frontier_detection_path, 'launch/frontier_detection_launch.py')),
            launch_arguments=[('log-level', logger)]
        )
    ]


    return launch.LaunchDescription(nodes)
