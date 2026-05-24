#!/usr/bin/env python3
import numpy as np
import sys
import math
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from rclpy.time import Time


from sensor_msgs.msg import PointCloud, CompressedImage, Image
from nav_msgs.msg import Odometry

from qr_destinations.destination_handler import DestinationHandler

# from std_msgs.msg import Header
# from geometry_msgs.msg import PoseWithCovarianceStamped

class RadarModuleNode(Node):
    def __init__(self) -> None:
        super().__init__("destination_advertiser")

        self.qr_handler = DestinationHandler(self)
    
        self._bridge = CvBridge()

        self.tf_collection_started = False
        self.tf_freshness_window = 50           # millis
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_check_timer = self.create_timer(0.05, self.get_map_tf)

        self.qr_location_subs = [
            Subscriber(self, PointCloud, 'pointcloud2d'),
            Subscriber(self, Image , '/camera/image_raw')
        ]

        self.qr_locating_time_sync = ApproximateTimeSynchronizer(
            self.qr_location_subs,
            queue_size=5,
            slop=0.2
        )

        self.qr_locating_time_sync.registerCallback(self.synced_qr_data_callback)

    def get_map_tf(self):
        target = 'odom'

        now = self.get_clock().now()
        
        try:
            t = self.tf_buffer.lookup_transform(target, 'base_link', Time())
        except TransformException as e:
            self.get_logger().warn(f"Failed to get {target} --> base_link")
            return
        
        if not self.tf_collection_started:
            self.tf_collection_started = True

        self.last_tf_time = self.millis_from_rclpy_time(now)
        self.last_tf = t

        return

    def synced_qr_data_callback(self, pointcloud_msg, image_msg):
        if not self.tf_collection_started:
            return

        if not self._is_fresh_tf():
            return  

        pts = np.array([[p.x, p.y] for p in pointcloud_msg.points], dtype=float)
        img = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        tf = self.last_tf.transform

        self.qr_handler.find_tags(pts, img, tf)
        pass

    def _is_fresh_tf(self):
        return True     

        tf_stamp_millis = (
            self.last_tf.header.stamp.sec * 1000.0 +
            self.last_tf.header.stamp.nanosec * 1e-6
        )
        now_millis = self.millis_from_rclpy_time(self.get_clock().now())

        age = now_millis - tf_stamp_millis
        if age > self.tf_freshness_window:
            self.get_logger().warn(f"Last tf is stale by "
                                   f"{(now_millis - tf_stamp_millis) - self.tf_freshness_window}"
                                   " millis")
            return False
        
        return True

    @staticmethod
    def millis_from_rclpy_time(rclpy_time):
        s_ns = rclpy_time.seconds_nanoseconds()

        return s_ns[0] * 1000.0 + s_ns[1] * 1e-6



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