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

# Imports
#* Import Python modules
import sys # System specific modules
import os # Operating specific functions
import glob
import time # Python timing module
import shutil # High level folder operation tool
from pathlib import Path # To find the "home" directory location
import argparse # To accept user arguments from commandline
# import natsort # To ensure all images are chosen loaded in the correct order
import yaml # To manipulate YAML files for reading configuration files
import copy # For making deepcopies of openCV matrices, python lists, numpy arrays etc.
import numpy as np # Python Linear Algebra module
import cv2 # OpenCV

#* ROS2 imports
import ament_index_python.packages
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# If you have more files in the submodules folder
# from .submodules.py_utils import fn1 # Import helper functions from files in your submodules folder

# Import a custom message interface
# from your_custom_msg_interface.msg import CustomMsg #* Note the camel caps convention

# Import ROS2 message templates
from sensor_msgs.msg import Image # http://wiki.ros.org/sensor_msgs
from std_msgs.msg import String, Float64 # ROS2 string message template
from cv_bridge import CvBridge, CvBridgeError # Library to convert image messages to numpy array
from diagnostic_msgs.msg import DiagnosticStatus
# Import lifecycle-related messages and services
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from ardupilot_msgs.msg import Status

#* Class definition
class MonoDriver(Node):
    def __init__(self, node_name = "mono_py_node"):
        super().__init__(node_name) # Initializes the rclpy.Node class. It expects the name of the node

        # Declare parameters that will be passed from a launch file or command line
        self.declare_parameter("settings_file_path","/ws/ros_ws/src/slam/config/orbslam3_mono_config.yaml")
        self.declare_parameter("camera_topic_name", "/camera")
        self.declare_parameter("vocab_file_path", "/ws/config/slam/orbslam3/vocab.txt")
        self.declare_parameter("slam_compute_node_name", "orbslam3_mono_node")
        self.declare_parameter("ardupilot_status_topic", "/ap/status")
        
        # Read the parameters
        self.settings_path = str(self.get_parameter('settings_file_path').value)
        self.camera_topic = str(self.get_parameter('camera_topic_name').value)
        self.vocab_file_path = str(self.get_parameter('vocab_file_path').value)
        self.slam_compute_node_name = str(self.get_parameter('slam_compute_node_name').value)
        self.ardupilot_status_topic = str(self.get_parameter('ardupilot_status_topic').value)

        # Log the received parameters for debugging and verification
        self.get_logger().info(f"-------------- Received parameters --------------------------")
        self.get_logger().info(f"settings_path: {self.settings_path}")
        self.get_logger().info(f"camera_topic: {self.camera_topic}")
        self.get_logger().info(f"vocab_file_path: {self.vocab_file_path}")
        self.get_logger().info(f"slam_compute_node_name: {self.slam_compute_node_name}")
        self.get_logger().info(f"ardupilot_status_topic: {self.ardupilot_status_topic}")
        self.get_logger().info(f"-------------------------------------------------------------")


        # Global variables
        self.node_name = node_name

        # Define a CvBridge object for processing images
        self.br = CvBridge()

        # Initialize work variables for main logic
        self.start_frame = 0 # Default 0
        self.end_frame = -1 # Default -1
        self.frame_stop = -1 # Set -1 to use the whole sequence, some positive integer to force sequence to stop, 350 test2, 736 test3
        self.show_imgs = False # Default, False, set True to see the output directly from this node
        self.frame_id = 0 # Integer id of an image frame
        self.frame_count = 0 # Ensure we are consistent with the count number of the frame
        self.inference_time = [] # List to compute average time

        self.get_logger().info(f"'{self.node_name}' initialized.")
        self.get_logger().info(f"Attempting handshake with lifecycle node '{self.slam_compute_node_name}'...")
        
        # --- Lifecycle Management ---
        # Create clients to manage the lifecycle of the SLAM node.
        self.service_callback_group =  rclpy.callback_groups.ReentrantCallbackGroup()
        change_state_service_name = f'/{self.slam_compute_node_name}/change_state'
        get_state_service_name = f'/{self.slam_compute_node_name}/get_state'
        self.change_state_client = self.create_client(ChangeState, change_state_service_name, callback_group=self.service_callback_group)
        self.get_state_client = self.create_client(GetState, get_state_service_name, callback_group=self.service_callback_group)

        # Wait for the lifecycle services to be available
        while not self.change_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(f"Lifecycle service '{change_state_service_name}' not available, waiting...")
        self.get_logger().info(f"Lifecycle service '{change_state_service_name}' now available")
        while not self.get_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(f"Lifecycle service '{get_state_service_name}' not available, waiting...")
        self.get_logger().info(f"Lifecycle service '{get_state_service_name}' now available")
        current_state = self._get_state()
        self.get_logger().info(f"Current state of node {self.slam_compute_node_name}: {current_state}")


        # --- ArduPilot Status Subscriber ---
        # Create a QoS profile that is compatible with the ArduPilot publisher
        status_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.status_subscriber = self.create_subscription(
            Status,
            self.ardupilot_status_topic,
            self._status_callback,
            status_qos_profile
        )
        self.get_logger().info(f"Subscribed to '{self.ardupilot_status_topic}'. Waiting for messages to control SLAM node...") 

    def _status_callback(self, msg: Status):
        """
        Callback for the ArduPilot status topic. Controls the SLAM node's lifecycle.
        """
        self.get_logger().debug(f"Received ArduPilot status: armed={msg.armed}, flying={msg.flying}")
        current_state = self._get_state()

        if current_state == -1:
            self.get_logger().warn("Could not determine state of SLAM node. Skipping status update.")
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
                 self.get_logger().info("Vehicle is ARMED and SLAM is inactive. Activating...")
                 self._change_state(Transition.TRANSITION_ACTIVATE)
            # If already active, do nothing.

        else: # --- DISARMING LOGIC ---
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
            self.get_logger().error(f"Failed to get state of '{self.slam_compute_node_name}' within {timeout_sec}s.")
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
            self.get_logger().error(f"Exception while calling '{self.change_state_client.srv_name}' service: {future.exception()}")
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
            mono_driver.get_logger().fatal(f"Unhandled exception in main: {e}", exc_info=True)
    finally:
        if executor:
            executor.shutdown()
        if mono_driver:
            mono_driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
diff --git a/src/controllers/controllers/slam/mono_driver_node.py b/src/controllers/controllers/slam/mono_driver_node.py
index ff826a1..7b8a0ae 100755
--- a/src/controllers/controllers/slam/mono_driver_node.py
+++ b/src/controllers/controllers/slam/mono_driver_node.py
@@ -26,7 +26,6 @@ import sys # System specific modules
 import os # Operating specific functions
 import glob
 import time # Python timing module
-import copy # For deepcopying arrays
 import shutil # High level folder operation tool
 from pathlib import Path # To find the "home" directory location
 import argparse # To accept user arguments from commandline
@@ -53,44 +52,46 @@ from sensor_msgs.msg import Image # http://wiki.ros.org/sensor_msgs
 from std_msgs.msg import String, Float64 # ROS2 string message template
 from cv_bridge import CvBridge, CvBridgeError # Library to convert image messages to numpy array
 from diagnostic_msgs.msg import DiagnosticStatus
-from custom_interfaces.srv import StartupSlam
+# Import lifecycle-related messages and services
+from lifecycle_msgs.srv import ChangeState, GetState
+from lifecycle_msgs.msg import Transition, State
+from ardupilot_msgs.msg import Status
 
 #* Class definition
 class MonoDriver(Node):
     def __init__(self, node_name = "mono_py_node"):
         super().__init__(node_name) # Initializes the rclpy.Node class. It expects the name of the node
 
-        self.declare_parameter("settings_path","/ws/ros_ws/src/slam/config/orbslam3_mono_config.yaml")
-        self.declare_parameter("camera_topic", "/camera")
+        # Declare parameters that will be passed from a launch file or command line
+        self.declare_parameter("settings_file_path","/ws/ros_ws/src/slam/config/orbslam3_mono_config.yaml")
+        self.declare_parameter("camera_topic_name", "/camera")
         self.declare_parameter("vocab_file_path", "/ws/config/slam/orbslam3/vocab.txt")
         self.declare_parameter("slam_compute_node_name", "orbslam3_mono_node")
-        # Initialize parameters to be passed from the command line (or launch file)
-        self.settings_path = str(self.get_parameter('settings_path').value) 
-        self.camera_topic = str(self.get_parameter('camera_topic').value)
+        self.declare_parameter("ardupilot_status_topic", "/ap/status")
+        
+        # Read the parameters
+        self.settings_path = str(self.get_parameter('settings_file_path').value)
+        self.camera_topic = str(self.get_parameter('camera_topic_name').value)
         self.vocab_file_path = str(self.get_parameter('vocab_file_path').value)
         self.slam_compute_node_name = str(self.get_parameter('slam_compute_node_name').value)
+        self.ardupilot_status_topic = str(self.get_parameter('ardupilot_status_topic').value)
 
-        self.slam_startup_client = self.create_client(StartupSlam, "/{}/start_slam".format(self.slam_compute_node_name))
-        while not self.slam_startup_client.wait_for_service(timeout_sec = 1.0):
-            self.get_logger().info('service not available, waiting again.....')
-        self.startup_req = StartupSlam.Request()
-        response = self.send_slam_startup_request(self.settings_path, self.camera_topic)
-        print(response.success)
-        #* Parse values sent by command line
+        # Log the received parameters for debugging and verification
+        self.get_logger().info(f"-------------- Received parameters --------------------------")
+        self.get_logger().info(f"settings_path: {self.settings_path}")
+        self.get_logger().info(f"camera_topic: {self.camera_topic}")
+        self.get_logger().info(f"vocab_file_path: {self.vocab_file_path}")
+        self.get_logger().info(f"slam_compute_node_name: {self.slam_compute_node_name}")
+        self.get_logger().info(f"ardupilot_status_topic: {self.ardupilot_status_topic}")
+        self.get_logger().info(f"-------------------------------------------------------------")
 
-        # DEBUG
-        print(f"-------------- Received parameters --------------------------\n")
-        print(f"settings_path: {self.settings_path}")
-        print(f"camera_topic: {self.camera_topic}")
-        print()
 
         # Global variables
-        self.node_name = "mono_py_driver"
+        self.node_name = node_name
 
         # Define a CvBridge object for processing images
         self.br = CvBridge()
 
-
         # Initialize work variables for main logic
         self.start_frame = 0 # Default 0
         self.end_frame = -1 # Default -1
@@ -100,15 +101,154 @@ class MonoDriver(Node):
         self.frame_count = 0 # Ensure we are consistent with the count number of the frame
         self.inference_time = [] # List to compute average time
 
-        print()
-        print(f"MonoDriver initialized, attempting handshake with CPP node")
+        self.get_logger().info(f"'{self.node_name}' initialized.")
+        self.get_logger().info(f"Attempting handshake with lifecycle node '{self.slam_compute_node_name}'...")
+        
+        # --- Lifecycle Management ---
+        # Create clients to manage the lifecycle of the SLAM node.
+        self.service_callback_group =  rclpy.callback_groups.ReentrantCallbackGroup()
+        change_state_service_name = f'/{self.slam_compute_node_name}/change_state'
+        get_state_service_name = f'/{self.slam_compute_node_name}/get_state'
+        self.change_state_client = self.create_client(ChangeState, change_state_service_name, callback_group=self.service_callback_group)
+        self.get_state_client = self.create_client(GetState, get_state_service_name, callback_group=self.service_callback_group)
+
+        # Wait for the lifecycle services to be available
+        while not self.change_state_client.wait_for_service(timeout_sec=5.0):
+            self.get_logger().info(f"Lifecycle service '{change_state_service_name}' not available, waiting...")
+        self.get_logger().info(f"Lifecycle service '{change_state_service_name}' now available")
+        while not self.get_state_client.wait_for_service(timeout_sec=5.0):
+            self.get_logger().info(f"Lifecycle service '{get_state_service_name}' not available, waiting...")
+        self.get_logger().info(f"Lifecycle service '{get_state_service_name}' now available")
+        current_state = self._get_state()
+        self.get_logger().info(f"Current state of node {self.slam_compute_node_name}: {current_state}")
+
+
+        # --- ArduPilot Status Subscriber ---
+        # Create a QoS profile that is compatible with the ArduPilot publisher
+        status_qos_profile = rclpy.qos.QoSProfile(
+            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
+            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
+            depth=1
+        )
+        self.status_subscriber = self.create_subscription(
+            Status,
+            self.ardupilot_status_topic,
+            self._status_callback,
+            status_qos_profile
+        )
+        self.get_logger().info(f"Subscribed to '{self.ardupilot_status_topic}'. Waiting for messages to control SLAM node...") 
+
+    def _status_callback(self, msg: Status):
+        """
+        Callback for the ArduPilot status topic. Controls the SLAM node's lifecycle.
+        """
+        self.get_logger().debug(f"Received ArduPilot status: armed={msg.armed}, flying={msg.flying}")
+        current_state = self._get_state()
+
+        if current_state == -1:
+            self.get_logger().warn("Could not determine state of SLAM node. Skipping status update.")
+            return
+
+        if msg.armed:
+            # --- ARMING LOGIC ---
+            if current_state == State.PRIMARY_STATE_UNCONFIGURED:
+                self.get_logger().info("Vehicle is ARMED. Configuring SLAM node...")
+                if self._change_state(Transition.TRANSITION_CONFIGURE):
+                    # After configuring, re-check state and activate
+                    if self._get_state() == State.PRIMARY_STATE_INACTIVE:
+                        self.get_logger().info("SLAM node configured. Activating...")
+                        self._change_state(Transition.TRANSITION_ACTIVATE)
+                else:
+                    self.get_logger().error("Failed to configure SLAM node on arm.")
+            elif current_state == State.PRIMARY_STATE_INACTIVE:
+                 self.get_logger().info("Vehicle is ARMED and SLAM is inactive. Activating...")
+                 self._change_state(Transition.TRANSITION_ACTIVATE)
+            # If already active, do nothing.
+
+        else: # --- DISARMING LOGIC ---
+            if current_state == State.PRIMARY_STATE_ACTIVE:
+                self.get_logger().info("Vehicle is DISARMED. Deactivating SLAM node...")
+                if self._change_state(Transition.TRANSITION_DEACTIVATE):
+                    # After deactivating, re-check state and cleanup
+                    if self._get_state() == State.PRIMARY_STATE_INACTIVE:
+                        self.get_logger().info("SLAM node deactivated. Cleaning up...")
+                        self._change_state(Transition.TRANSITION_CLEANUP)
+                else:
+                    self.get_logger().error("Failed to deactivate SLAM node on disarm.")
+            # elif current_state == State.PRIMARY_STATE_INACTIVE:
+            #     self.get_logger().info("Vehicle is DISARMED and SLAM is inactive. Cleaning up...")
+            #     self._change_state(Transition.TRANSITION_CLEANUP)
+            # If already unconfigured, do nothing.
+
     # ****************************************************************************************
+    def _get_state(self, timeout_sec=5.0) -> int:
+        """
+        Gets the current state of the lifecycle node.
+        :param timeout_sec: Timeout for the service call.
+        :return: The current state ID (e.g., State.PRIMARY_STATE_UNCONFIGURED) or -1 on failure.
+        """
+        request = GetState.Request()
+        future = self.get_state_client.call_async(request)
+        
+        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
+        
+        if future.result():
+            return future.result().current_state.id
+        else:
+            self.get_logger().error(f"Failed to get state of '{self.slam_compute_node_name}' within {timeout_sec}s.")
+            return -1
 
     # ****************************************************************************************
-    def send_slam_startup_request(self, settings_path, camera_topic):
-        self.startup_req.config_file_path = settings_path
-        self.startup_req.camera_topic = camera_topic
-        self.startup_req.camera_type = 'monocular'
-        self.slam_startup_future = self.slam_startup_client.call_async(self.startup_req)
-        rclpy.spin_until_future_complete(self, self.slam_startup_future)
-        return self.slam_startup_future.result()
+    def _change_state(self, transition_id: int) -> bool:
+        """
+        Calls the change_state service of the lifecycle node to request a state transition.
+        :param transition_id: The ID of the transition to request (e.g., Transition.TRANSITION_CONFIGURE).
+        :return: True if the transition was successful, False otherwise.
+        """
+        request = ChangeState.Request()
+        request.transition.id = transition_id
+        
+        # Call the service asynchronously
+        future = self.change_state_client.call_async(request)
+        
+        # Wait until the service call is complete
+        rclpy.spin_until_future_complete(self, future)
+        
+        # Check the result of the service call
+        if future.result() is not None:
+            if future.result().success:
+                return True
+            else:
+                self.get_logger().warn(f"Transition '{transition_id}' failed.")
+                return False
+        else:
+            self.get_logger().error(f"Exception while calling '{self.change_state_client.srv_name}' service: {future.exception()}")
+            return False
+
+# You would typically have a main function here to spin up the node
+def main(args=None):
+    rclpy.init(args=args)
+
+    mono_driver = None
+    executor = None
+    try:
+        mono_driver = MonoDriver()
+        # A MultiThreadedExecutor is used to process callbacks from different callback groups in parallel.
+        executor = MultiThreadedExecutor()
+        executor.add_node(mono_driver)
+        executor.spin()
+    except KeyboardInterrupt:
+        pass
+    except Exception as e:
+        if mono_driver:
+            mono_driver.get_logger().fatal(f"Unhandled exception in main: {e}", exc_info=True)
+    finally:
+        if executor:
+            executor.shutdown()
+        if mono_driver:
+            mono_driver.destroy_node()
+        if rclpy.ok():
+            rclpy.shutdown()
+
+if __name__ == '__main__':
+    main()
diff --git a/src/controllers/launch/target_recorder.launch.py b/src/controllers/launch/target_recorder.launch.py
index 1b3365b..870bf5b 100644
--- a/src/controllers/launch/target_recorder.launch.py
+++ b/src/controllers/launch/target_recorder.launch.py
@@ -21,7 +21,7 @@ def generate_launch_description():
     output_bag_name_arg = launch.actions.DeclareLaunchArgument(
         'output_bag_name',
         default_value='/ws/data/telemetry',
-        description='Name of the output rosbag file.'
+        description='Name of the output rosbag file. The output bag name must not contain special symbols and numbers.'
     )
 
     # Define the list of all supported topics.
@@ -29,7 +29,8 @@ def generate_launch_description():
         "/ap/airspeed", "/ap/battery", "/ap/clock", "/ap/cmd_gps_pose", "/ap/cmd_vel",
         "/ap/geopose/filtered", "/ap/goal_lla", "/ap/gps_global_origin/filtered",
         "/ap/imu/experimental/data", "/ap/joy", "/ap/navsat", "/ap/pose/filtered",
-        "/ap/status", "/ap/tf", "/ap/tf_static", "/ap/time", "/ap/twist/filtered"
+        "/ap/status", "/ap/tf", "/ap/tf_static", "/ap/time", "/ap/twist/filtered",
+        "/tf", "/tf_static"
     ]
     # Format the list into a string that looks like a Python list, e.g., "['/topic1', '/topic2']"
     # This is the format the ROS 2 parameter system expects for a vector of strings.
diff --git a/src/flight_matrix_bridge b/src/flight_matrix_bridge
index b81ecf6..a2822f3 160000
--- a/src/flight_matrix_bridge
+++ b/src/flight_matrix_bridge
@@ -1 +1 @@
-Subproject commit b81ecf60fb95acc2145674288b2f5b2cfd2b66a6
+Subproject commit a2822f3af527e79359e53fa9c4be4248bfe4d320-dirty
diff --git a/src/mpu6050 b/src/mpu6050
index 804c704..9084974 160000
--- a/src/mpu6050
+++ b/src/mpu6050
@@ -1 +1 @@
-Subproject commit 804c704e652c313885dc18ea882e892fcddae144
+Subproject commit 9084974c64795507c4b6db45778dbe1e76c5415d
diff --git a/src/sensors/include/sensors/common/utils.h b/src/sensors/include/sensors/common/utils.h
index 5e2f9bd..a9b5a43 100644
--- a/src/sensors/include/sensors/common/utils.h
+++ b/src/sensors/include/sensors/common/utils.h
@@ -10,6 +10,7 @@
 #include <string>
 #include <vector>
 #include <map>
+#include <filesystem>
 
 typedef struct {
 	int mWidth;
diff --git a/src/sensors/include/sensors/logger/managed_logger.h b/src/sensors/include/sensors/logger/managed_logger.h
index 599d26d..b63df03 100644
--- a/src/sensors/include/sensors/logger/managed_logger.h
+++ b/src/sensors/include/sensors/logger/managed_logger.h
@@ -55,6 +55,10 @@ class ManagedLogger : public rclcpp_lifecycle::LifecycleNode
 		 * @brief Reads and validates parameters from the parameter server.
 		 */
 		void InitializeParameters();
+		/**
+		 * @brief Checks if the provided bagfile path exists and updates the output file path with an incremental ID
+		 */
+		bool UpdateFilenameIfExists(std::string& filepath);
 
 		/// @brief A shared pointer to the underlying rosbag2 C++ writer instance.
 		std::shared_ptr<rosbag2_cpp::Writer> mpWriter;
diff --git a/src/sensors/src/logger/managed_logger.cpp b/src/sensors/src/logger/managed_logger.cpp
index 58de8e7..ab6c690 100644
--- a/src/sensors/src/logger/managed_logger.cpp
+++ b/src/sensors/src/logger/managed_logger.cpp
@@ -132,10 +132,50 @@ CallbackReturn ManagedLogger::on_cleanup(const rclcpp_lifecycle::State &)
 	return CallbackReturn::SUCCESS;
 }
 
+bool ManagedLogger::UpdateFilenameIfExists(std::string& filepath) {
+    if (std::filesystem::exists(filepath)) {
+        size_t last_underscore_pos = filepath.rfind('_');
+
+        if (last_underscore_pos == std::string::npos) {
+			// First replacement ID starts with 1
+            filepath += "_1";
+        } else {
+            try {
+                std::string number_str = filepath.substr(last_underscore_pos + 1);
+                int current_number = std::stoi(number_str);
+                int new_number = current_number + 1;
+                // Create the new filename by replacing the old number with the new one.
+                filepath = filepath.substr(0, last_underscore_pos + 1) + std::to_string(new_number);
+            } catch (const std::invalid_argument& ia) {
+                // This catch block handles cases where the substring after '_' is not a valid number.
+                // For example, "/ws/data/telemetry_". In this case, we can append "1".
+                filepath += "1";
+            } catch (const std::out_of_range& oor) {
+                // This handles cases where the number is too large to fit in an int.
+                RCLCPP_ERROR(this->get_logger(), "Error: Number out of range in filename: %s", filepath.c_str());
+            }
+        }
+		if(!UpdateFilenameIfExists(filepath)){
+			RCLCPP_INFO(this->get_logger(), "Provided filename exists, logging at: %s", filepath.c_str());
+		}
+		return true;
+    }
+	else{
+		return false;
+	}
+    // If the file does not exist, the filepath remains unchanged.
+}
+
 void ManagedLogger::InitializeParameters()
 {
 	this->get_parameter("output_bag_name", mpOutputBagName);
 	this->get_parameter("topics_to_record", mvpTopicsToRecord);
+	UpdateFilenameIfExists(mpOutputBagName);
+
+	if(std::filesystem::exists(mpOutputBagName)){
+		RCLCPP_WARN(this->get_logger(), "bag with specified name exists, adding a number in the name to distinguish");
+
+	}
 
 	if (mvpTopicsToRecord.empty()) {
 		RCLCPP_ERROR(this->get_logger(), "Parameter 'topics_to_record' is not set or is empty.");
diff --git a/src/slam b/src/slam
index 4ec3104..3d074b2 160000
--- a/src/slam
+++ b/src/slam
@@ -1 +1 @@
-Subproject commit 4ec3104e0e308d13e5eee786ca48549dfdfd7582
+Subproject commit 3d074b24c46e217aa687360a250fa9cca10b0a4f
diff --git a/src/tello b/src/tello
--- a/src/tello
+++ b/src/tello
@@ -1 +1 @@
-Subproject commit 9d756450c7b04a71ee296c22d81ff0b21e9d91c0
+Subproject commit 9d756450c7b04a71ee296c22d81ff0b21e9d91c0-dirty
diff --git a/src/vision-opencv b/src/vision-opencv
index 066793a..9800f67 160000
--- a/src/vision-opencv
+++ b/src/vision-opencv
@@ -1 +1 @@
-Subproject commit 066793a23e5d06d76c78ca3d69824a501c3554fd
+Subproject commit 9800f67cea477c44cfb64e349854bcb6a09dc9ce
