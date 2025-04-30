import launch
import launch_ros

def generate_launch_description():
    logger_default = "INFO"
    logger = launch.substitutions.LaunchConfiguration("log-level", default = logger_default)
    nodes = [
        launch_ros.actions.Node(
            package="controllers",
            executable="video_test_controller",
            output="screen",
            namespace="/",
            name="test_controller",
            respawn=False,
            arguments=['--ros-args', '--log-level', logger]
        ),
        launch_ros.actions.Node(
            package='sensors',
            executable='video_logger_node',
            output='screen',
            namespace='/',
            name='video_logger',
            respawn=False,
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
        launch_ros.actions.Node(
            package='sensors',
            executable='video_reader_node',
            output='screen',
            namespace='/',
            name='video_reader',
            respawn=False,
            arguments=['--ros-args', '--log-level', logger],
            prefix = ['gdb --ex run --ex bt --args'],
            parameters = [{
                'camera_topic' : "/camera",
                'input_file_path' : '/ws/data/test.mp4'
            }],
        ),
    ]

    return launch.LaunchDescription(nodes)
