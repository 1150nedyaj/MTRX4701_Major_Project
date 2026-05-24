#!/usr/bin/env python3
import numpy as np
import sys
import math
from cv_bridge import CvBridge


import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import PointCloud, CompressedImage, Image
from nav_msgs.msg import Odometry

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped


from qr_destinations.destination_handler import DestinationHandler


class RadarModuleNode(Node):
    def __init__(self) -> None:
        super().__init__("destination_advertiser")

        self.qr_handler = DestinationHandler(self)
    
        self._bridge = CvBridge()


        self.qr_location_subs = [
            Subscriber(self, PointCloud, 'pointcloud2d'),
            Subscriber(self, Image , '/camera/image_raw'),
            Subscriber(self, Odometry, 'odom')
        ]

        self.qr_locating_time_sync = ApproximateTimeSynchronizer(
            self.qr_location_subs,
            queue_size=5,
            slop=0.2
        )

        self.qr_locating_time_sync.registerCallback(self.synced_qr_data_callback)

    def synced_qr_data_callback(self, pointcloud_msg, image_msg, odom_msg):

        pts = np.array([[p.x, p.y] for p in pointcloud_msg.points], dtype=float)
        img = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        pose = odom_msg.pose

        self.qr_handler.find_tags(pts, img, pose)
        pass



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