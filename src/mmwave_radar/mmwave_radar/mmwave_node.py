import rclpy
from rclpy.node import Node


class MmwaveNode(Node):
    def __init__(self):
        super().__init__("mmwave_node")
        self.get_logger().info("mmWave radar node started")


def main(args=None):
    rclpy.init(args=args)
    node = MmwaveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
