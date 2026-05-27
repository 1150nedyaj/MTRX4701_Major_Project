#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from destination_msgs.msg import DestinationListMsg
from geometry_msgs.msg import PoseStamped


class GoalSenderNode(Node):
    def __init__(self) -> None:
        super().__init__("goal_sender")

        self.declare_parameter('goal_frame', 'map')
        self._goal_frame = self.get_parameter('goal_frame').value

        self.create_subscription(
            DestinationListMsg,
            '/destination_advertiser/list',
            self.destinations_callback,
            3
        )

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 3)

    def destinations_callback(self, msg):
        if not msg.destinations:
            self.get_logger().warn("Received empty destination list, nothing to send")
            return

        target = msg.destinations[0]

        goal = PoseStamped()
        goal.header.frame_id = self._goal_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose = target.pose

        label = target.name if target.name else f'tag {target.tag}'
        self.get_logger().info(
            f"Sending goal for '{label}' -> "
            f"x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}"
        )

        self.goal_pub.publish(goal)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalSenderNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()