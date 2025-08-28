import launch
import launch_ros

def generate_launch_description():
    log_level_arg = launch.actions.DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        description="Logging level for the nodes (DEBUG, INFO, WARN, ERROR, FATAL)"
    )
    log_level = launch.substitutions.LaunchConfiguration("log_level")
    ns_arg = launch.actions.DeclareLaunchArgument(
        "drone_name",
        default_value="drone1",
        description="Name of the Tello drone for ID"
    )
    ns = launch.substitutions.LaunchConfiguration("drone_name")
    sensors_pkg_path = launch_ros.substitutions.FindPackageShare('sensors')
    encoder_launch_file = launch.substitutions.PathJoinSubstitution([
        sensors_pkg_path,
        'launch',
        'ffmpeg_encode.launch.py'
    ])
    encoder_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(encoder_launch_file),
        launch_arguments={
            'input_topic': launch.substitutions.PythonExpression(["'/' + ","'", ns, "' + '/image_raw'"]),
            'ffmpeg_topic': launch.substitutions.PythonExpression(["'/' + ","'", ns, "' + '/image_compressed'"])}.items()
    )
    topics_str = launch.substitutions.PythonExpression([
            "['/' + '", ns, "' + '/image_compressed', ",
            "'/' + '", ns, "' + '/flight_data',",
            "'/' + '", ns, "' + '/joy',",
            "'/' + '", ns, "' + '/tello_state',",
            "'/' + '", ns, "' + '/cmd_vel',",
            "'/' + '", ns, "' + '/camera_info']"])
    # topics_str = f"[{', '.join([f'{repr(topic)}' for topic in all_supported_topics_list])}]"
    controllers_pkg_path = launch_ros.substitutions.FindPackageShare('controllers')
    logger_launch_file = launch.substitutions.PathJoinSubstitution([
        controllers_pkg_path,
        'launch',
        'target_recorder.launch.py'
    ])
    logger_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(logger_launch_file),
        launch_arguments={
            'topics_to_record': topics_str,
            'output_bag_name': '/ws/data/telemetry',
            'log_level': log_level,
            'ap_status_topic_name': launch.substitutions.PythonExpression(["'/' + ","'", ns, "' + '/tello_state'"]),
            'ap_type': 'tello'}.items()
    )
    # TODO Need to make sure all namepsaces here are parameters and this launch file is reusable with any configuration
    # TODO Remove hardocded topic names from within the controller and tello joy stick node
    return launch.LaunchDescription([
        log_level_arg,
        # TelloControllerNode with remapped topics
        launch_ros.actions.Node(
            package="controllers",
            executable="tello_controller_node",
            name="tello_controller",
            namespace=ns,
            output="screen",
            parameters = [{
                "drone_name": ns,
                }],
            arguments=['--ros-args', '--log-level', [log_level]],
            # prefix=['gdb -ex run -ex bt --args']
            ),
        # Lifecycle node for Tello Joy
        launch_ros.actions.LifecycleNode(
            package='tello_driver',
            executable='tello_joy_main',
            name='tello_joy',
            parameters = [{
                "drone_name": ns,
                }],
            namespace=ns,
            output='screen',
            ),
        # Joy node with consistent remapping
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace=ns,
            output='screen',

            ),
        launch_ros.actions.Node(
            package='tello_driver',
            executable='tello_driver_main',
            name='tello_driver_main',
            namespace=ns,
            output='screen',
            ),
        encoder_launch,
        logger_launch
    ])
