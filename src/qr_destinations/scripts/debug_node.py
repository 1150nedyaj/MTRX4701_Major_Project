#!/usr/bin/env python3

"""
MTRX4701 2026 Assignment 4
File: debug_node.py
Author(s): 530 499 451

Used for displaying the destinations being advertised when the destination_node is being
run on the turtlebot.
"""

import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node

from destination_msgs.msg import DestinationListMsg

class RadarModuleNode(Node):
    def __init__(self) -> None:
        super().__init__("destination_debug")

        self.create_subscription(DestinationListMsg, '/destination_advertiser/list', self.destinations_callback, 3)

        plt.ion()
        self._fig, self._ax = plt.subplots(1, 1, figsize=(8, 8))
        self._fig.suptitle('Destination Monitor')

    def destinations_callback(self, msg):
        self.get_logger().info('callback fired')
        self._update_plot(msg.destinations)


    def _update_plot(self, destinations):
        self._ax.cla()
        self._ax.set_title('Tracked Destinations')
        self._ax.set_xlabel('x (m)')
        self._ax.set_ylabel('y (m)')
        self._ax.set_aspect('equal')
        self._ax.grid(True)

        for dest in destinations:
            gx = dest.pose.position.x
            gy = dest.pose.position.y

            self._ax.scatter(gx, gy, s=100, c='blue', zorder=3)

            yaw = self._yaw_from_quaternion(dest.pose.orientation)
            self._ax.annotate(
                '', xy=(gx + 0.15 * np.cos(yaw), gy + 0.15 * np.sin(yaw)), xytext=(gx, gy),
                arrowprops=dict(arrowstyle='->', color='tomato', lw=1.5)
            )

            label = dest.name if dest.name else f'tag {dest.tag}'
            self._ax.text(gx + 0.03, gy + 0.03, label, fontsize=9)

        self._fig.tight_layout()
        self._fig.canvas.draw()
        self._fig.canvas.flush_events()

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        """Extract yaw (rotation about Z) from a geometry_msgs Quaternion."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return float(np.arctan2(siny_cosp, cosy_cosp))


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