#!/usr/bin/env python3


"""
Python node for configuring and driving the MonocularMode cpp node.
Proprocessing may ne performed in this step.
The node may dispatch the received data for logging.

Author: Azmyin Md. Kamal
Date: 01/01/2024

Requirements
* Dataset must be configured in EuRoC MAV format
* Paths to dataset must be set before bulding (or running) this node
* Make sure to set path to your workspace in common.hpp

Command line arguments
-- settings_name: EuRoC, TUM2, KITTI etc; the name of the .yaml file containing camera intrinsics and other configurations
-- image_seq: MH01, V102, etc; the name of the image sequence you want to run

"""

import argparse  # To accept user arguments from commandline
import copy  # For making deepcopies of openCV matrices, python lists, numpy arrays etc.
import glob
import os  # Operating specific functions
import shutil  # High level folder operation tool

# Imports
# * Import Python modules
import sys  # System specific modules
import time  # Python timing module
from pathlib import Path  # To find the "home" directory location

# * ROS2 imports
import ament_index_python.packages
import cv2  # OpenCV
import numpy as np  # Python Linear Algebra module
import rclpy

# import natsort # To ensure all images are chosen loaded in the correct order
import yaml  # To manipulate YAML files for reading configuration files
from ardupilot_msgs.msg import Status
from cv_bridge import (  # Library to convert image messages to numpy array
    CvBridge,
    CvBridgeError,
)
from diagnostic_msgs.msg import DiagnosticStatus
from lifecycle_msgs.msg import State, Transition

# Import lifecycle-related messages and services
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.parameter import Parameter

# Import ROS2 message templates
from sensor_msgs.msg import Image  # http://wiki.ros.org/sensor_msgs
from std_msgs.msg import Float64, String  # ROS2 string message template

# If you have more files in the submodules folder
# from .submodules.py_utils import fn1 # Import helper functions from files in your submodules folder

# Import a custom message interface
# from your_custom_msg_interface.msg import CustomMsg #* Note the camel caps convention


# * Class definition
class MonoDriver(Node):
    def __init__(self, node_name="mono_py_node"):
        super().__init__(
            node_name
        )  # Initializes the rclpy.Node class. It expects the name of the node

        # Declare parameters that will be passed from a launch file or command line
        self.declare_parameter(
            "settings_file_path", "/ws/ros_ws/src/slam/config/orbslam3_mono_config.yaml"
        )
        self.declare_parameter("camera_topic_name", "/camera")
        self.declare_parameter("vocab_file_path", "/ws/config/slam/orbslam3/vocab.txt")
        self.declare_parameter("slam_compute_node_name", "orbslam3_mono_node")
        self.declare_parameter("ardupilot_status_topic", "/ap/status")
        self.declare_parameter("tello_status_topic", "/tello_state")
        drone_type_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="Type of drone supported by the SLAM driver. Can be one of- 'ardupilot' or 'tello'",
        )
        self.declare_parameter("drone_type", "ardupilot", drone_type_descriptor)

        # Read the parameters
        self.drone_type = str(self.get_parameter("drone_type").value)
        self.settings_path = str(self.get_parameter("settings_file_path").value)
        self.camera_topic = str(self.get_parameter("camera_topic_name").value)
        self.vocab_file_path = str(self.get_parameter("vocab_file_path").value)
        self.slam_compute_node_name = str(
            self.get_parameter("slam_compute_node_name").value
        )
        self.ardupilot_status_topic = str(
            self.get_parameter("ardupilot_status_topic").value
        )
        self.tello_status_topic = str(self.get_parameter("tello_status_topic").value)

        # Log the received parameters for debugging and verification
        self.get_logger().info(
            f"-------------- Received parameters --------------------------"
        )
        self.get_logger().info(f"settings_path: {self.settings_path}")
        self.get_logger().info(f"camera_topic: {self.camera_topic}")
        self.get_logger().info(f"vocab_file_path: {self.vocab_file_path}")
        self.get_logger().info(f"slam_compute_node_name: {self.slam_compute_node_name}")
        self.get_logger().info(f"ardupilot_status_topic: {self.ardupilot_status_topic}")
        self.get_logger().info(f"tello_status_topic: {self.tello_status_topic}")
        self.get_logger().info(f"drone_type: {self.drone_type}")
        self.get_logger().info(
            f"-------------------------------------------------------------"
        )

        # Global variables
        self.node_name = node_name

        # Define a CvBridge object for processing images
        self.br = CvBridge()

        # Initialize work variables for main logic
        self.start_frame = 0  # Default 0
        self.end_frame = -1  # Default -1
        self.frame_stop = (
            -1
        )  # Set -1 to use the whole sequence, some positive integer to force sequence to stop, 350 test2, 736 test3
        self.show_imgs = (
            False  # Default, False, set True to see the output directly from this node
        )
        self.frame_id = 0  # Integer id of an image frame
        self.frame_count = (
            0  # Ensure we are consistent with the count number of the frame
        )
        self.inference_time = []  # List to compute average time

        self.get_logger().info(f"'{self.node_name}' initialized.")
        self.get_logger().info(
            f"Attempting handshake with lifecycle node '{self.slam_compute_node_name}'..."
        )

        # --- Lifecycle Management ---
        # Create clients to manage the lifecycle of the SLAM node.
        self.service_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        change_state_service_name = f"/{self.slam_compute_node_name}/change_state"
        get_state_service_name = f"/{self.slam_compute_node_name}/get_state"
        self.change_state_client = self.create_client(
            ChangeState,
            change_state_service_name,
            callback_group=self.service_callback_group,
        )
        self.get_state_client = self.create_client(
            GetState, get_state_service_name, callback_group=self.service_callback_group
        )

        # Wait for the lifecycle services to be available
        while not self.change_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(
                f"Lifecycle service '{change_state_service_name}' not available, waiting..."
            )
        self.get_logger().info(
            f"Lifecycle service '{change_state_service_name}' now available"
        )
        while not self.get_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(
                f"Lifecycle service '{get_state_service_name}' not available, waiting..."
            )
        self.get_logger().info(
            f"Lifecycle service '{get_state_service_name}' now available"
        )
        current_state = self._get_state()
        self.get_logger().info(
            f"Current state of node {self.slam_compute_node_name}: {current_state}"
        )

        # --- ArduPilot Status Subscriber ---
        # Create a QoS profile that is compatible with the ArduPilot publisher
        status_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        status_topic = ""
        if self.drone_type == "ardupilot":
            status_topic = self.ardupilot_status_topic
            self.status_subscriber = self.create_subscription(
                Status,
                status_topic,
                self._ap_status_callback,
                status_qos_profile,
            )
        elif self.drone_type == "tello":
            status_topic = self.tello_status_topic
            self.status_subscriber = self.create_subscription(
                String,
                status_topic,
                self._tello_status_callback,
                status_qos_profile,
            )
        self.get_logger().info(
            f"Subscribed to '{status_topic}'. Waiting for messages to control SLAM node..."
        )

    def _tello_status_callback(self, msg: String):
        """
        Callback for the Tello status topic. Controls the SLAM node's lifecycle.
        """
        self.get_logger().info(f"Received ArduPilot status: {msg.data}")
        current_state = self._get_state()

        if current_state == -1:
            self.get_logger().warn(
                "Could not determine state of SLAM node. Skipping status update."
            )
            return

        if msg.data == "taking off":
            # --- ARMING LOGIC ---
            if current_state == State.PRIMARY_STATE_UNCONFIGURED:
                self.get_logger().info("Vehicle is ARMED. Configuring SLAM node...")
                if self._change_state(Transition.TRANSITION_CONFIGURE):
                    # After configuring, re-check state and activate
                    if self._get_state() == State.PRIMARY_STATE_INACTIVE:
                        self.get_logger().info("SLAM node configured. Activating...")
                        self._change_state(Transition.TRANSITION_ACTIVATE)
                else:
                    self.get_logger().error("Failed to configure SLAM node on arm.")
            elif current_state == State.PRIMARY_STATE_INACTIVE:
                self.get_logger().info(
                    "Vehicle is ARMED and SLAM is inactive. Activating..."
                )
                self._change_state(Transition.TRANSITION_ACTIVATE)
            # If already active, do nothing.
        elif msg.data == "landed":
            if current_state == State.PRIMARY_STATE_ACTIVE:
                self.get_logger().info("Vehicle is DISARMED. Deactivating SLAM node...")
                if self._change_state(Transition.TRANSITION_DEACTIVATE):
                    # After deactivating, re-check state and cleanup
                    if self._get_state() == State.PRIMARY_STATE_INACTIVE:
                        self.get_logger().info("SLAM node deactivated. Cleaning up...")
                        self._change_state(Transition.TRANSITION_CLEANUP)
                else:
                    self.get_logger().error("Failed to deactivate SLAM node on disarm.")

    def _ap_status_callback(self, msg: Status):
        """
        Callback for the ArduPilot status topic. Controls the SLAM node's lifecycle.
        """
        self.get_logger().debug(
            f"Received ArduPilot status: armed={msg.armed}, flying={msg.flying}"
        )
        current_state = self._get_state()

        if current_state == -1:
            self.get_logger().warn(
                "Could not determine state of SLAM node. Skipping status update."
            )
            return

        if msg.armed:
            # --- ARMING LOGIC ---
            if current_state == State.PRIMARY_STATE_UNCONFIGURED:
                self.get_logger().info("Vehicle is ARMED. Configuring SLAM node...")
                if self._change_state(Transition.TRANSITION_CONFIGURE):
                    # After configuring, re-check state and activate
                    if self._get_state() == State.PRIMARY_STATE_INACTIVE:
                        self.get_logger().info("SLAM node configured. Activating...")
                        self._change_state(Transition.TRANSITION_ACTIVATE)
                else:
                    self.get_logger().error("Failed to configure SLAM node on arm.")
            elif current_state == State.PRIMARY_STATE_INACTIVE:
                self.get_logger().info(
                    "Vehicle is ARMED and SLAM is inactive. Activating..."
                )
                self._change_state(Transition.TRANSITION_ACTIVATE)
            # If already active, do nothing.

        else:  # --- DISARMING LOGIC ---
            if current_state == State.PRIMARY_STATE_ACTIVE:
                self.get_logger().info("Vehicle is DISARMED. Deactivating SLAM node...")
                if self._change_state(Transition.TRANSITION_DEACTIVATE):
                    # After deactivating, re-check state and cleanup
                    if self._get_state() == State.PRIMARY_STATE_INACTIVE:
                        self.get_logger().info("SLAM node deactivated. Cleaning up...")
                        self._change_state(Transition.TRANSITION_CLEANUP)
                else:
                    self.get_logger().error("Failed to deactivate SLAM node on disarm.")
            # elif current_state == State.PRIMARY_STATE_INACTIVE:
            #     self.get_logger().info("Vehicle is DISARMED and SLAM is inactive. Cleaning up...")
            #     self._change_state(Transition.TRANSITION_CLEANUP)
            # If already unconfigured, do nothing.

    # ****************************************************************************************
    def _get_state(self, timeout_sec=5.0) -> int:
        """
        Gets the current state of the lifecycle node.
        :param timeout_sec: Timeout for the service call.
        :return: The current state ID (e.g., State.PRIMARY_STATE_UNCONFIGURED) or -1 on failure.
        """
        request = GetState.Request()
        future = self.get_state_client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if future.result():
            return future.result().current_state.id
        else:
            self.get_logger().error(
                f"Failed to get state of '{self.slam_compute_node_name}' within {timeout_sec}s."
            )
            return -1

    # ****************************************************************************************
    def _change_state(self, transition_id: int) -> bool:
        """
        Calls the change_state service of the lifecycle node to request a state transition.
        :param transition_id: The ID of the transition to request (e.g., Transition.TRANSITION_CONFIGURE).
        :return: True if the transition was successful, False otherwise.
        """
        request = ChangeState.Request()
        request.transition.id = transition_id

        # Call the service asynchronously
        future = self.change_state_client.call_async(request)

        # Wait until the service call is complete
        rclpy.spin_until_future_complete(self, future)

        # Check the result of the service call
        if future.result() is not None:
            if future.result().success:
                return True
            else:
                self.get_logger().warn(f"Transition '{transition_id}' failed.")
                return False
        else:
            self.get_logger().error(
                f"Exception while calling '{self.change_state_client.srv_name}' service: {future.exception()}"
            )
            return False


# You would typically have a main function here to spin up the node
def main(args=None):
    rclpy.init(args=args)

    mono_driver = None
    executor = None
    try:
        mono_driver = MonoDriver()
        # A MultiThreadedExecutor is used to process callbacks from different callback groups in parallel.
        executor = MultiThreadedExecutor()
        executor.add_node(mono_driver)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if mono_driver:
            mono_driver.get_logger().fatal(
                f"Unhandled exception in main: {e}",
            )
    finally:
        if executor:
            executor.shutdown()
        if mono_driver:
            mono_driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
