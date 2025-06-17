from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    ns = 'drone1'
    sensors_pkg_path = FindPackageShare('sensors')
    encoder_launch_file = PathJoinSubstitution([
        sensors_pkg_path,
        'launch',
        'ffmpeg_encode.launch.py'
    ])
    encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(encoder_launch_file),
        launch_arguments={
            'input_topic': '/image_raw',
            'ffmpeg_topic': '/camera/compressed'}.items()
    )
    all_supported_topics_list = [
        '/camera/compressed'
    ]
    topics_str = f"[{', '.join([f'{repr(topic)}' for topic in all_supported_topics_list])}]"
    logger_launch_file = PathJoinSubstitution([
        sensors_pkg_path,
        'launch',
        'managed_logger.launch.py'
    ])
    logger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(logger_launch_file),
        launch_arguments={
            'topics_to_record': topics_str,
            'output_bag_name': '/ws/data/telemetry'}.items()
    )
    return LaunchDescription([
        # TelloControllerNode with remapped topics
        Node(
            package="controllers",
            executable="tello_controller_node",
            name="tello_controller",
            namespace=ns,
            output="screen",
            ),
        # Lifecycle node for Tello Joy
        LifecycleNode(
            package='tello_driver',
            executable='tello_joy_main',
            name='tello_joy',
            namespace=ns,
            output='screen',
            ),
        # Joy node with consistent remapping
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace=ns,
            output='screen',

            ),

        Node(
            package='tello_driver',
            executable='tello_driver_main',
            name='tello_driver_main',
            namespace=ns,
            output='screen',
            ),
        encoder_launch,
        logger_launch
    ])
