#!/usr/bin/env python3

"""
MTRX4701 2026 Assignment 4
File: radar_module_node.py
Author(s): Jeremy Fox

This node exposes a single mmWave radar module to the ROS middleware, 
handing configuration parameters through to the actual RadarModuleHandler class. 

While it does publish a custom message for radar detections through the '/detections' topic,
it also pushes them through '/pose' as a Pose With Covariance. This pose topic is just intended 
to be used for debugging, while the custom message gets used in any computation.

All information about how the module is orientated is handled by the array_transforms_node.
"""


import numpy as np
import sys
import math

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped
from radar_messages.msg import StampedRadarDetections, RadarDetection

from mmwave_radar.radar_module_handler import RadarModuleHandler


class RadarModuleNode(Node):
    """
    Sets up a radar, passing through id and UART interface
    """

    def __init__(self) -> None:
        super().__init__("radar_module_node")

        self.declare_parameter('identifier', rclpy.Parameter.Type.INTEGER)
        self.node_id = self.get_parameter('identifier').get_parameter_value().integer_value
        assert self.node_id is not None
        assert self.node_id >= 0

        self.declare_parameter('interface', rclpy.Parameter.Type.STRING)
        self.serial_interface = self.get_parameter('interface').get_parameter_value().string_value
        assert self.serial_interface is not None
        assert len(self.serial_interface) > 0

        self.module_frame = f'radar{self.node_id}'
        self.radar_handler = RadarModuleHandler(self, self.serial_interface)

        self.reading_publish_period = 0.15 # s -> sensor sample freq. is 10Hz
        self._radar_report_pub = self.create_publisher(StampedRadarDetections, "~/detections", 1)
        self._radar_detect_debug_pub = self.create_publisher(PoseWithCovarianceStamped, "~/pose", 20)
        self._radar_pub_timer = self.create_timer(self.reading_publish_period, self.pub_radar_detections)
        

    def pub_radar_detections(self):

        signatures = self.radar_handler.get_signatures()
        
        if len(signatures) == 0:
            # bail from publish
            return

        report_msg = StampedRadarDetections()

        # header contents
        report_msg.header = Header()
        report_msg.header.stamp = self.get_clock().now().to_msg()
        report_msg.header.frame_id = self.module_frame

        # report data
        for s in signatures:
            
            # self.get_logger().info(f"{s}")
            self.pub_radar_pose(s)

            detection_msg = RadarDetection()

            detection_msg.position.x = float(s.x)
            detection_msg.position.y = float(s.y)
            detection_msg.position.z = 0.0

            detection_msg.speed = float(s.speed)

            detection_msg.covariance = s.covariance.flatten().tolist()

            report_msg.detections.append(detection_msg)


        self._radar_report_pub.publish(report_msg)


    def pub_radar_pose(self, s):
        # debug
        pose_msg = PoseWithCovarianceStamped()

        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = f'radar{self.node_id}'

        pose_msg.pose.pose.position.x = float(s.x)
        pose_msg.pose.pose.position.y = float(s.y)
        pose_msg.pose.pose.position.z = 0.0
        
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0

        covariance = np.zeros((6,6))
        covariance[0:2, 0:2] = s.covariance
        covariance = np.block([
            [s.covariance, np.zeros((2,4))],
            [np.zeros((4,3)), np.zeros((4,3))]
        ])
        pose_msg.pose.covariance = covariance.flatten().tolist()

        self._radar_detect_debug_pub.publish(pose_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RadarModuleNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()