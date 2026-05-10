#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from radar_messages.msg import StampedReport

from mmwave_radar.radar_module_handler import RadarModuleHandler
from mmwave_radar.types import RadarFrame


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
        self._radar_report_pub = self.create_publisher(StampedReport, "~/report", 1)
        self._radar_pub_timer = self.create_timer(self.reading_publish_period, self.pub_radar_report)
    
    def pub_radar_report(self):
        report_data = self.radar_handler.read_radar_data()
        # self.get_logger().info(f"Data -> {report_data}")
        if report_data is None:
            # bail from publish
            return

        report_msg = StampedReport()

        # header contents
        report_msg.header = Header()
        report_msg.header.stamp = self.get_clock().now().to_msg()
        report_msg.header.frame_id = f'radar{self.node_id}'

        # report data
        report_msg.distance = float(report_data.distance if report_data.distance is not None else 0.0)
        for i, eV in enumerate(report_data.gate_energies):
            report_msg.gate_energies[i] = int(eV)

        self._radar_report_pub.publish(report_msg)



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