import os

import ament_index_python
import launch_ros

import launch


def generate_launch_description():
    log_level_default = "INFO"
    log_level = launch.substitutions.LaunchConfiguration(
        "log_level", default=log_level_default
    )
    log_level_arg = launch.actions.DeclareLaunchArgument(
        "log_level", default_value=[log_level_default], description="Logging level"
    )

    controllers_launch_dir = os.path.join(
        ament_index_python.packages.get_package_share_directory("controllers"), "launch"
    )

    sitl_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(controllers_launch_dir, "sitl.launch.py")
        ),
        launch_arguments={
            "log_level": log_level,
        }.items(),
    )

    slam_launch_dir = os.path.join(
        ament_index_python.packages.get_package_share_directory("slam"), "launch"
    )

    basalt_slam_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(slam_launch_dir, "basalt_slam.launch.py")
        ),
        launch_arguments={
            "log-level": logger,
        }.items(),
    )

    delayed_seq_launch = launch.actions.TimerAction(
        period=0.0, actions=[sitl_launch, basalt_slam_launch]
    )

    nodes = [
        log_level_arg,
        delayed_seq_launch,
    ]

    return launch.LaunchDescription(nodes)
