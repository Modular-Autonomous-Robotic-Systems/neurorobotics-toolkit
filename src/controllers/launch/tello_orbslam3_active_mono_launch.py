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
        
        launch_ros.actions.Node(
            package='sensors',
            executable='video_logger_node',
            output='screen',
            namespace='/',
            name='video_logger',
            respawn=False,
            prefix = ['gdb -batch -ex run -ex bt --args'],
            arguments=['--ros-args', '--log-level', logger],
            parameters = [{
                'output_file_path': '/ws/data/output.mp4',
                'width' : 640,
                'height' : 480,
                'fps' : 30,
                'camera_topic' : '/camera',
                'image_type' : 'bgr'
            }],
        ),

        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen'
        ),

        # OctoMap Builder Node
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(frontier_detection_path, 'launch/frontier_detection_launch.py')),
            launch_arguments=[('log-level', logger)]
        )
    ]


    return launch.LaunchDescription(nodes)
