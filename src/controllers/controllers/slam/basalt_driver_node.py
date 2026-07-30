#!/usr/bin/env python3
"""Python node for configuring and driving the Basalt SLAM cpp node (VSLAM / VISLAM)."""

import rclpy
from controllers.slam.driver_node import SLAMDriverNode
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor


class BasaltSLAMDriver(SLAMDriverNode):
    def __init__(self, node_name="mono_py_node"):
        super().__init__(node_name)
        self.declare_parameter(
            "config_file_path", "/ws/ros_ws/src/slam/ext/basalt/data/sitl_config.json"
        )
        self.declare_parameter("camera_topic_name", "/camera")
        self.declare_parameter("imu_topic_name", "/imu")
        self.declare_parameter(
            "calib_file_path", "/ws/src/slam/ext/basalt/data/sitl_calib.json"
        )
        self.config_path = str(self.get_parameter("config_file_path").value)
        self.camera_topic = str(self.get_parameter("camera_topic_name").value)
        self.imu_topic = str(self.get_parameter("imu_topic_name").value)
        self.calib_file_path = str(self.get_parameter("calib_file_path").value)
        self.get_logger().info(
            f"-------------- Received SLAM parameters --------------------------"
        )
        self.get_logger().info(f"config_path: {self.config_path}")
        self.get_logger().info(f"camera_topic: {self.camera_topic}")
        self.get_logger().info(f"calib_file_path: {self.calib_file_path}")
        self.get_logger().info(
            f"-------------------------------------------------------------"
        )
        self.node_name = node_name
        self.br = CvBridge()
        self.start_frame = 0
        self.end_frame = -1
        self.frame_stop = -1
        self.show_imgs = False
        self.frame_id = 0
        self.frame_count = 0
        self.inference_time = []


def main(args=None):
    basalt_driver = None
    executor = None
    try:
        rclpy.init(args=args)
        basalt_driver = BasaltSLAMDriver()
        executor = MultiThreadedExecutor()
        executor.add_node(basalt_driver)
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if basalt_driver is not None:
            basalt_driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
