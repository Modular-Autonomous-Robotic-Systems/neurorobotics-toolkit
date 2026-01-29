# Sensors Package

## Project Overview

This is a ROS 2 package named `sensors` designed to handle sensor data logging and playback, with a focus on video and IMU data. It integrates with AirSim, OpenCV, and GStreamer.

### Key Components

*   **Video Logger (`video_logger_node`):** Subscribes to a camera topic and saves the video stream to a file (e.g., MP4) using OpenCV and GStreamer.
*   **Video Reader (`video_reader_node`):** Reads a video file and publishes it to a ROS 2 topic.
*   **IMU Logger (`imu_logger_node`):** Logs IMU data.
*   **Managed Logger (`managed_logger_node`):** A `rclcpp_lifecycle` managed node that wraps `rosbag2` to record specific topics. It supports dynamic configuration of the output bag name and topics to record.
*   **Managed Player (`managed_player_node`):** A lifecycle managed node for playing back data (likely using `rosbag2` mechanisms, similar to the logger).
*   **AirSim Integration:** Contains an `ext/AirSim` directory, suggesting integration with the AirSim simulator.

## Building and Running

### Build

This package uses `ament_cmake`. To build the package:

```bash
colcon build --packages-select sensors
```

To build with tests:

```bash
colcon build --packages-select sensors --cmake-args -DBUILD_TESTING=ON
```

### Run

**Video Logging and Reading Test:**

```bash
ros2 launch sensors video_test.launch.py
```

**Managed Logger:**

You can run the managed logger directly or via a launch file (if available). It requires lifecycle management commands to configure and activate.

```bash
ros2 run sensors managed_logger_node --ros-args -p output_bag_name:=my_bag -p topics_to_record:=['/topic1','/topic2']
```

Then, use `ros2 lifecycle` to manage the state:

```bash
ros2 lifecycle set /managed_logger_node configure
ros2 lifecycle set /managed_logger_node activate
```

## Development Conventions

*   **Language:** C++ (C++14/17 standards implied by ROS 2).
*   **Build System:** CMake with `ament_cmake`.
*   **Code Style:** Follows standard ROS 2 C++ style.
*   **Lifecycle Nodes:** The `managed_logger_node` and `managed_player_node` use the `rclcpp_lifecycle` framework, requiring implementation of `on_configure`, `on_activate`, `on_deactivate`, and `on_cleanup`.
*   **Dependencies:**
    *   ROS 2 Core (`rclcpp`, `std_msgs`, `sensor_msgs`)
    *   CV Bridge & OpenCV (`cv_bridge`, `image_transport`, `OpenCV`)
    *   Rosbag2 (`rosbag2_cpp`, `rosbag2_transport`)
    *   GStreamer

## Commit Message Guidelines

All commits affecting this package must follow the project-wide conventions:

*   **Structure**: `Subject` -> `Problem` -> `Solution` -> `Note`.
*   **Method References**: Always include the class name (e.g., `ManagedLoggerNode::on_configure`).
*   **File Paths**: Use full relative paths (e.g., `src/sensors/launch/managed_logger.launch.py`).
*   **Wrapping**: Wrap text at 72 characters.
