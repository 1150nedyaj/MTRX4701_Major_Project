#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped
from radar_messages.msg import StampedRadarDetections, RadarDetection

from mmwave_radar.radar_module_handler import RadarModuleHandler


class RadarModuleNode(Node):
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


        self.radar_handler = RadarModuleHandler(self, self.serial_interface)

        self.reading_publish_period = 0.15 # s -> sensor sample freq. is 10Hz
        self._radar_report_pub = self.create_publisher(StampedRadarDetections, "~/detections", 1)
        self._radar_detect_debug_pub = self.create_publisher(PoseWithCovarianceStamped, "~/pose", 5)
        self._radar_pub_timer = self.create_timer(self.reading_publish_period, self.pub_radar_detections)
        


    def pub_radar_detections(self):
        signatures = self.radar_handler.get_signatures()
        
        # self.get_logger().info(f"Data -> {report_data}")
        if len(signatures) == 0:
            # bail from publish
            return

        report_msg = StampedRadarDetections()

        # header contents
        report_msg.header = Header()
        report_msg.header.stamp = self.get_clock().now().to_msg()
        report_msg.header.frame_id = f'radar{self.node_id}'

        # report data

        # X AND Y ARE SWITCHED FOR RADAR
        for s in signatures:
            
            self.get_logger().info(f"{s}")
            self.pub_radar_pose(s)

            detection_msg = RadarDetection()

            detection_msg.x = float(s.x)
            detection_msg.y = float(s.y)
            detection_msg.speed = float(s.speed)
            report_msg.detections.append(detection_msg)


        self._radar_report_pub.publish(report_msg)

    def pub_radar_pose(self, s):
        # debugf
        pose_msg = PoseWithCovarianceStamped()

        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = f'radar{self.node_id}'

        pose_msg.pose.pose.position.x = float(s.y)/1000
        pose_msg.pose.pose.position.y = float(s.x)/1000
        pose_msg.pose.pose.position.z = 0.0
        
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 0.0

        covariance = np.block([
            [s.covariance, np.zeros((2,4))],
            [np.zeros((4,3)), np.zeros((4,3))]
        ])


        # covariance = [0.09, 0.0, 0.0, 0.0, 0.0, 0.0,  # x
        #             0.0, 0.09, 0.0, 0.0, 0.0, 0.0,  # y
        #             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # z
        #             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # roll
        #             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # pitch
        #             0.0, 0.0, 0.0, 0.0, 0.0, 0.1]   # yaw
        pose_msg.pose.covariance = covariance

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