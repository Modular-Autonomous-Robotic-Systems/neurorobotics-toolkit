# Launch Files

The following launch files are available in the package:
1. `target_recorder.launch.py`: This launch file starts the video controller node and the `managed_logger.launch.py` launch file in `sensors` package.
    - The following launch arguments are supported:
        - `log_level`: Set to `INFO` by default and may take one of `DEBUG`, `INFO`, `WARN`, `ERROR`, `FATAL`
        - `ap_status_topic_name`: Topic of autopilot state publisher. May be one of `/ap/status` and `/<ns>/tello_state`
        - `ap_type`: Type of autopilot to to record data for. May be on of `ardupilot` and `tello`
        - `topics_to_record`: List of all topics to record with launch file
        - `output_bag_name`: Path to directory to log ROS2 bags
    - To record compressed data the ffmpeg encoder in `sensors` package must be launched.
2. `sitl.launch.py`: The launch file starts SITL with compressed video logging.
    - The following launch arguments are supported:
        - `log_level`: Set to `INFO` by default and may take one of `DEBUG`, `INFO`, `WARN`, `ERROR`, `FATAL`
        - `ap_status_topic_name`: Topic name to be provided to managed logger. This must be `/ap/status` by default unless otherwise changed from within ardupilot
    - This launch file uses `target_recorder.launch.py` to record data
    - This launch file used `ffmpeg_encode.launch.py` from `sensors` package to encode data before logging with SITL
3. `tello_controller.launch.py`: This launch file starts the tello driver for joystick control of the tello
    - The following launch arguments are supported:
        - `log-level`: Set to `INFO` by default and may take one of `DEBUG`, `INFO`, `WARN`, `ERROR`, `FATAL`
        - `drone-name`: Name of the tello drone namespace (default: `drone1`)
        - `use-joystick`: Boolean argument to toggle the use of joystick to control tello drone (default: `true`)
    - The launch file uses `target_recorder.launch.py` to record data
    - This launch file used `ffmpeg_encode.launch.py` from `sensors` package to encode data before logging during tello flights
4. `ardupilot_target.launch.py`: Generates a launch description that wraps the ArduPilot SITL micro-ROS agent launch file.
    - The following launch arguments are supported:
        - `verbosity`: Set the verbosity level for logging (0-6) (default: `4`)
        - `device`: The device path for the serial connection (default: `/dev/ttyS0`)
        - `baudrate`: The baud rate for the serial connection (default: `115200`)
        - `ap_status_topic_name`: Topic name for ArduPilot status messages (default: `/ap/status`)
5. `basalt_slam_test.launch.py`: Launches SITL and Basalt SLAM.
    - The following launch arguments are supported:
        - `log_level`: Logging level (default: `INFO`)
6. `replay_active_mono.launch.py`: Replays telemetry data and runs ORB-SLAM3 controller alongside frontier detection.
    - The following launch arguments are supported:
        - `log-level`: Logging level for the nodes (default: `INFO`)
        - `drone-name`: Name of the Tello drone for ID (default: `drone1`)
        - `telemetry_path`: Path to the ros2 bag to replay (default: `/ws/data/telemetry`)
7. `slam_controller.launch.py`: Starts the ORB-SLAM3 mono node and controller.
    - The following launch arguments are supported:
        - `log-level`: Logging level (default: `INFO`)
        - `camera-topic`: Camera topic name to listen to for images (default: `/camera`)
        - `settings-file-path`: Path to ORBSLAM3 settings file (default: `/ws/ros_ws/src/slam/orb_slam3/config/Monocular/sitl.yaml`)
        - `vocab-file-path`: Path to ORBSLAM3 vocabulary file (default: `/ws/ros_ws/src/slam/orb_slam3/Vocabulary/ORBvoc.txt.bin`)
        - `drone-type`: Type of drone platform to start SLAM for (default: `ardupilot`)
        - `arduilot-status-topic-name`: Topic Name for SLAM controller to listen to for ArduPilot Status (default: `/ap/status`)
        - `tello-status-topic-name`: Topic Name for SLAM controller to listen to for tello Status (default: `/tello_state`)
8. `tello_orbslam3_active_mono_launch.py`: Launches Tello controller, ORB-SLAM3, and frontier detection for active mapping.
    - The following launch arguments are supported:
        - `log-level`: Logging level for the nodes (default: `INFO`)
        - `drone-name`: Name of the Tello drone for ID (default: `drone1`)
9. `tello_orbslam3_mono_launch.py`: Launches Tello driver, ORB-SLAM3 node and controller with video encoding and logging.
    - The following launch arguments are supported:
        - `log-level`: Logging level (default: `INFO`)
10. `video.launch.py`: Starts the `video_logger_test` and its `VideoLoggingDriver`.
    - The following launch arguments are supported:
        - `log_level`: Logging level for the nodes (default: `INFO`)
        - `ap_status_topic_name`: Topic name for ArduPilot status messages (default: `/ap/status`)