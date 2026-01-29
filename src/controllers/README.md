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
        - `log_level`: Set to `INFO` by default and may take one of `DEBUG`, `INFO`, `WARN`, `ERROR`, `FATAL`
        - `drone_name`: Name of the tello drone namespace
    - The launch file uses `target_recorder.launch.py` to record data
    - This launch file used `ffmpeg_encode.launch.py` from `sensors` package to encode data before logging during tello flights
