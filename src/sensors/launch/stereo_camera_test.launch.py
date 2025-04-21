from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Declare launch argument for shared parameter file
    Sample camera_params.yaml

    This YAML file configures both the left and right camera nodes.
    It must follow the ROS 2 structured format with node names and ros__parameters.

    stereo/left/left_camera:
      ros__parameters:
        camera: 0
        width: 640
        height: 480
        format: RGB
        camera_info_url: file:///absolute/path/to/left_camera_calibration.yaml

    stereo/right/right_camera:
      ros__parameters:
        camera: 1
        width: 640
        height: 480
        format: RGB
        camera_info_url: file:///absolute/path/to/right_camera_calibration.yaml
    """
    declare_shared_param_file = DeclareLaunchArgument(
        'camera_param_file',
        default_value='/path/to/camera_params.yaml',
        description='Path to YAML file containing parameters for both camera nodes'
    )

    # Composable node container for left and right camera nodes
    camera_container = ComposableNodeContainer(
        name='stereo_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='camera_ros',
                plugin='camera::CameraNode',
                name='left_camera',
                namespace='stereo/left',
                parameters=[LaunchConfiguration('camera_param_file')]
            ),
            ComposableNode(
                package='camera_ros',
                plugin='camera::CameraNode',
                name='right_camera',
                namespace='stereo/right',
                parameters=[LaunchConfiguration('camera_param_file')]
            ),
        ],
        output='screen'
    )

    # Standard node for stereo image processing (disparity computation)
    stereo_image_proc_node = Node(
        package='stereo_image_proc',
        executable='stereo_image_proc',
        name='stereo_image_proc',
        namespace='stereo',
        remappings=[
            ('left/image_rect', 'left/image_raw'),
            ('right/image_rect', 'right/image_raw'),
            ('left/camera_info', 'left/camera_info'),
            ('right/camera_info', 'right/camera_info')
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_shared_param_file,
        camera_container,
        stereo_image_proc_node
    ])

