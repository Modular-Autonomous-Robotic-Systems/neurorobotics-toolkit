import os
import launch
import launch_ros

def generate_launch_description():
    # --- ArduPilot SITL Launch ---
    # Assuming workspace overlay of /ardupilot_ws
    ardupilot_sitl_pkg_path = launch_ros.substitutions.FindPackageShare('ardupilot_sitl')
    ardupilot_launch_file = launch.substitutions.PathJoinSubstitution([
        ardupilot_sitl_pkg_path,
        'launch',
        'sitl_dds_udp.launch.py'
    ])

    # Construct the full path for the parameter files
    default_copter_params = launch.substitutions.PathJoinSubstitution([
        ardupilot_sitl_pkg_path,
        'config',
        'default_params',
        'copter.parm'
    ])
    
    default_dds_udp_params = launch.substitutions.PathJoinSubstitution([
        ardupilot_sitl_pkg_path,
        'config',
        'default_params',
        'dds_udp.parm'
    ])

    ardupilot_sitl_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(ardupilot_launch_file),
        launch_arguments={
            'transport': 'udp4',
            'synthetic_clock': 'True',
            'wipe': 'False',
            'model': 'quad',
            'speedup': '1',
            'slave': '0',
            'instance': '0',
            'defaults': [default_copter_params, ',', default_dds_udp_params],
            'sim_address': '0.0.0.0',
            'master': 'tcp:0.0.0.0:5760',
            'sitl': '0.0.0.0:5501',
            'map': 'False',
            'console': 'False',
            'sim_port_in': '9003',
            'sim_port_out': '9002'
        }.items()
    )

    return launch.LaunchDescription([
        ardupilot_sitl_launch
    ])
