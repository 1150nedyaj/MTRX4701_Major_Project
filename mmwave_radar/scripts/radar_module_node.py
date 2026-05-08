#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# from std_msgs.msg import Int32Stamped

from mmwave_radar.radar_module_handler import RadarModuleHandler


class RadarModuleNode(Node):
    def __init__(self) -> None:
        super().__init__("radar_module_node_0")

        self.reading_publish_period = 0.1 # s -> sensor sample freq. is 10Hz
        self.radar_handler = RadarModuleHandler(self, '/dev/ttyAMA0')

        self.radar_pub_timer = self.create_timer(self.reading_publish_period, self.pub_radar)
    
    def pub_radar(self):
        self.get_logger().info("Radar Publish Callback Fired.")
        self.get_logger().info(f"Reading -> {self.radar_handler.read_radar_data()}")


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