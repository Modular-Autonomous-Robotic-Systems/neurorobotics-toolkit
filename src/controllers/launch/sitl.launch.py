import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    delayed_launch = TimerAction(
        period=3.0,  # Seconds
        actions=[
            LogInfo(msg="Timer elapsed. Launching airsim_ros_pkgs node"),
            # Action to launch airsim_ros_pkgs (defined below)
            # This will be added to the actions list of TimerAction
        ]
    )
    # --- AirSim Launch ---
    # Assuming workspace overlay of /airsim_ws
    airsim_ros_pkg_path = FindPackageShare('airsim_ros_pkgs')
    airsim_launch_file = PathJoinSubstitution([
        airsim_ros_pkg_path,
        'launch',
        'airsim_node.launch.py'
    ])

    airsim_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(airsim_launch_file),
        launch_arguments={'host_ip': '127.0.0.1'}.items()
    )
    delayed_launch.actions.append(airsim_node_launch)

    # --- ArduPilot SITL Launch ---
    # Assuming workspace overlay of /ardupilot_ws
    ardupilot_sitl_pkg_path = FindPackageShare('ardupilot_sitl')
    ardupilot_launch_file = PathJoinSubstitution([
        ardupilot_sitl_pkg_path,
        'launch',
        'sitl_dds_udp.launch.py'
    ])

    # Construct the full path for the parameter files
    default_copter_params = PathJoinSubstitution([
        ardupilot_sitl_pkg_path,
        'config',
        'default_params',
        'copter.parm'
    ])
    
    default_airsim_copter_params = PathJoinSubstitution([
        ardupilot_sitl_pkg_path,
        'config',
        'default_params',
        'airsim-quadX.parm'
    ])

    default_dds_udp_params = PathJoinSubstitution([
        ardupilot_sitl_pkg_path,
        'config',
        'default_params',
        'dds_udp.parm'
    ])

    ardupilot_sitl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ardupilot_launch_file),
        launch_arguments={
            'transport': 'udp4',
            'synthetic_clock': 'True',
            'wipe': 'False',
            'model': 'airsim-copter',
            'speedup': '1',
            'slave': '0',
            'instance': '0',
            'defaults': [default_copter_params, ',', default_dds_udp_params, ',', default_airsim_copter_params], # Note: passing list for multiple files
            'sim_address': '127.0.0.1', # This was duplicated in your command, ensuring one is passed
            'master': 'tcp:127.0.0.1:5760',
            'sitl': '127.0.0.1:5501',
            'map': 'False',
            'console': 'False',
            'sim_port_in': '9003',
            'sim_port_out': '9002'
            # 'sim_address': '127.0.0.1' # Already included
        }.items()
    )
    delayed_launch.actions.append(
        LogInfo(msg="Launching ardupilot_sitl after airsim_ros_pkgs.")
    )
    delayed_launch.actions.append(ardupilot_sitl_launch)
    
    # TODO need to install appropriate gstreamer dependencies in sitl_bridge_container
    # # --- Video Logger Launch ---
    # controllers_pkg_path = FindPackageShare('controllers')
    # video_logger_launch_file = PathJoinSubstitution([
    #     controllers_pkg_path,
    #     'launch',
    #     'video.launch.py'
    # ])
    # video_logger_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(video_logger_launch_file),
    #     launch_arguments={
    #         'log_level': "INFO",
    #         'ap_status_topic_name': "/ap/status"
    #     }.items()
    # )
    # delayed_launch.actions.append(
    #     LogInfo(msg="Launching video_logger after ardupilot_sitl.")
    # )
    # delayed_launch.actions.append(video_logger_launch)

    return LaunchDescription([
        LogInfo(msg="Assuming AirSim simulator is starting/started externally."),
        LogInfo(msg="Introducing a 3-second delay before launching airsim_ros_pkgs."),
        delayed_launch
    ])
