import launch
import launch_ros

def generate_launch_description() -> launch.LaunchDescription:
    """
    Generate a launch description for the camera node.

    This launch file starts a single camera node with configurable
    parameters for camera ID, image dimensions, topic, and format.

    Returns
    -------
        launch.LaunchDescription: The launch description.

    """
    camera_id_arg = launch.actions.DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='ID of the camera to start streaming from.'
    )
    image_width_arg = launch.actions.DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Width of the camera image.'
    )
    image_height_arg = launch.actions.DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Height of the camera image.'
    )
    camera_topic_arg = launch.actions.DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera',
        description='Base topic name to publish the camera streams.'
    )
    image_format_arg = launch.actions.DeclareLaunchArgument(
        'image_format',
        default_value='auto',
        description='Pixel format of the image to publish.'
    )

    # 2. Get the launch configurations
    camera_id_config = launch.substitutions.LaunchConfiguration('camera_id')
    image_width_config = launch.substitutions.LaunchConfiguration('image_width')
    image_height_config = launch.substitutions.LaunchConfiguration('image_height')
    camera_topic_config = launch.substitutions.LaunchConfiguration('camera_topic')
    image_format_config = launch.substitutions.LaunchConfiguration('image_format')

    # 3. Define the ComposableNode for the camera
    camera_node = launch_ros.descriptions.ComposableNode(
        package='camera_ros',
        plugin='camera::CameraNode',
        name='camera_node',
        parameters=[{
            'camera': camera_id_config,
            'width': image_width_config,
            'height': image_height_config,
            'format': image_format_config,
        }],
        # Remap the default '/camera' topic to the user-specified topic
        remappings=[
            ('/camera', camera_topic_config)
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # 4. Define the ComposableNodeContainer to run the node. Added 
    camera_container = launch_ros.actions.ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            camera_node
        ],
        output='screen',
    )

    # 5. Create the launch description and add the components
    ld = launch.LaunchDescription()
    ld.add_action(camera_id_arg)
    ld.add_action(image_width_arg)
    ld.add_action(image_height_arg)
    ld.add_action(camera_topic_arg)
    ld.add_action(image_format_arg)
    ld.add_action(camera_container)

    return ld

