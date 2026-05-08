import rclpy
from rclpy.node import Node


class MovementNode(Node):
    def __init__(self):
        super().__init__("movement_node")
        self.get_logger().info("Robot movement node started")


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

