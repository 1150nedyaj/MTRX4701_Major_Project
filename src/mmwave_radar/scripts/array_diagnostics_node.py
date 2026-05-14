#!/usr/bin/env python3
import sys


import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

from std_msgs.msg import Header
from radar_messages.msg import StampedReport

from mmwave_radar.types import RadarFrame
from mmwave_radar.array_diagnostics_display import ArrayDiagnosticsDisplay


class ArrayDiagnosticsNode(Node):
    def __init__(self) -> None:
        super().__init__("array_diagnostics_node")
        
        # Radar Report Queue Setup
        queue_size = 20
        slop = 0.15  # seconds; radar reports come in @ 10Hz

        # find all the radar reports being published
        available_topics = self.get_topic_names_and_types()
        self.radar_topics = []
        for tN, _ in available_topics:
            components = tN.split('/')[1:]  # 0th item always empty

            if components[0] == 'mmWave_array' and components[-1] == 'report':
                self.radar_topics.append(tN)

        if len(self.radar_topics) == 0:
            self.get_logger().error(f"No mmWave_array nodes publishing reports.")
            sys.exit(1)
        else:
            self.get_logger().info(f"### Found {len(self.radar_topics)} Radar Report Topics ###")

        # setup handling of message from all the radar reports
        self.report_subs = [
            Subscriber(self, StampedReport, topic) for topic in self.radar_topics
        ]

        self.report_time_sync = ApproximateTimeSynchronizer(
            self.report_subs,
            queue_size,
            slop,
        )
        self.report_time_sync.registerCallback(self._synced_report_callback)

        # plotting (scheming even)
        self.plotter = ArrayDiagnosticsDisplay(radar_count=len(self.radar_topics))
    
    def _synced_report_callback(self, *reports):
        frames = [r.header.frame_id for r in reports]
        self.get_logger().info(
            f"Synced bundle of {len(reports)} reports from: {frames}"
        )

        t = self.get_clock().now().nanoseconds
        self.plotter.routine(reports, t)




def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArrayDiagnosticsNode()
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