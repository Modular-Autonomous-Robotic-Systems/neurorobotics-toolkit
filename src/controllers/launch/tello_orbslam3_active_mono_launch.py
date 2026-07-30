import os

import ament_index_python.packages
import launch
import launch_ros


def generate_launch_description():
    logger_default = "INFO"
    logger = launch.substitutions.LaunchConfiguration(
        "log-level",
        default=logger_default,
    )
    logger_arg = launch.actions.DeclareLaunchArgument(
        "log-level",
        default_value=[logger_default],
        description="Logging level for the nodes (DEBUG, INFO, WARN, ERROR, FATAL)",
    )
    ns_arg = launch.actions.DeclareLaunchArgument(
        "drone-name",
        default_value="drone1",
        description="Name of the Tello drone for ID",
    )
    ns = launch.substitutions.LaunchConfiguration("drone-name")
    camera_topic = (
        launch.substitutions.PythonExpression(["'/' + ", "'", ns, "' + '/image_raw'"]),
    )
    tello_status_topic = (
        launch.substitutions.PythonExpression(
            ["'/' + ", "'", ns, "' + '/tello_state'"]
        ),
    )
    controllers_pkg_path = ament_index_python.packages.get_package_share_directory(
        "controllers"
    )
    frontier_detection_path = ament_index_python.packages.get_package_share_directory(
        "frontier_detection"
    )
    nodes = [
        # Tello driver node
        logger_arg,
        ns_arg,
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(controllers_pkg_path, "launch/tello_controller.launch.py")
            ),
            launch_arguments=[
                ("log_level", logger),
                ("drone_name", ns),
                ("use_joystick", "true"),
            ],
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(controllers_pkg_path, "launch/slam_controller.launch.py")
            ),
            launch_arguments=[
                ("log-level", logger),
                ("camera-topic", camera_topic),
                (
                    "settings-file-path",
                    "/ws/ros_ws/src/slam/orb_slam3/config/Monocular/tello.yaml",
                ),
                (
                    "vocab-file-path",
                    "/ws/ros_ws/src/slam/orb_slam3/Vocabulary/ORBvoc.txt.bin",
                ),
                ("drone-type", "tello"),
                ("tello-status-topic-name", tello_status_topic),
            ],
        ),
        # OctoMap Builder Node
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    frontier_detection_path, "launch/frontier_detection_launch.py"
                )
            ),
            launch_arguments=[("log-level", logger)],
        ),
    ]

    return launch.LaunchDescription(nodes)
