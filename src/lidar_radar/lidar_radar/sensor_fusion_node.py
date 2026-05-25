#!/usr/bin/env python3
"""
Sensor fusion: confirm lidar people detections with radar.

A lidar-detected person is forwarded only when a radar detection exists
within a configurable spatial and temporal window.  This eliminates false
positives that arise from circular static objects (traffic cones, poles,
wheels) which the circle-detector cannot distinguish from legs.

Subscribes
----------
/lidar/circle_candidates  (geometry_msgs/PoseArray)
    People positions from the lidar circle-detector, in the scan frame.
<radar_topic>  (radar_messages/StampedRadarDetections)  [one or more]
    Raw radar detections, in each module's own TF frame.
    Default: ["/mmWave_array/radar_0/detections"]
    Override via the "radar_topics" parameter (string array).

Publishes
---------
/fusion/people            (geometry_msgs/PoseArray)
    Subset of lidar detections confirmed by radar, same frame as input.
/fusion/people_markers    (visualization_msgs/MarkerArray)
    Green cylinders for confirmed people (RViz visualisation).

Parameters
----------
radar_topics              string[]  – radar detection topic names
fusion_distance_threshold double    – max distance (m) for a radar point to
                                      confirm a lidar detection  [default 1.0]
radar_timeout             double    – keep radar readings for this many seconds
                                      [default 0.5]
target_frame              string    – common TF frame for spatial comparison
                                      [default "base_link"]
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import Buffer, TransformListener, TransformException

from radar_messages.msg import StampedRadarDetections


class SensorFusionNode(Node):

    def __init__(self):
        super().__init__("sensor_fusion")

        # --- parameters ---
        self.declare_parameter("radar_topics", ["/mmWave_array/radar_0/detections"])
        self.declare_parameter("fusion_distance_threshold", 1.0)
        self.declare_parameter("radar_timeout", 0.5)
        self.declare_parameter("target_frame", "base_link")

        radar_topics = (
            self.get_parameter("radar_topics")
            .get_parameter_value()
            .string_array_value
        )
        self._threshold = (
            self.get_parameter("fusion_distance_threshold")
            .get_parameter_value()
            .double_value
        )
        self._radar_timeout = (
            self.get_parameter("radar_timeout").get_parameter_value().double_value
        )
        self._target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )

        # --- TF ---
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- radar buffer: deque of (wall_time_sec, [(x, y), ...]) ---
        # Points are already transformed into _target_frame when stored.
        self._radar_buffer: deque = deque()

        # --- radar subscribers (one per topic) ---
        self._radar_subs = []
        for topic in radar_topics:
            sub = self.create_subscription(
                StampedRadarDetections, topic, self._radar_cb, 10
            )
            self._radar_subs.append(sub)
            self.get_logger().info(f"Subscribed to radar: {topic}")

        # --- lidar subscriber ---
        self._lidar_sub = self.create_subscription(
            PoseArray, "/lidar/circle_candidates", self._lidar_cb, 10
        )

        # --- publishers ---
        self._people_pub = self.create_publisher(PoseArray, "/fusion/people", 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, "/fusion/people_markers", 10
        )

        self.get_logger().info(
            f"sensor_fusion ready | threshold={self._threshold} m | "
            f"timeout={self._radar_timeout} s | frame={self._target_frame}"
        )

    # ------------------------------------------------------------------
    # Radar callback
    # ------------------------------------------------------------------

    def _radar_cb(self, msg: StampedRadarDetections):
        """Transform incoming radar detections into target_frame and buffer them."""
        if not msg.detections:
            return

        source_frame = msg.header.frame_id

        # Look up the transform once for the whole message.
        tf = self._get_transform(source_frame, msg.header.stamp)
        if tf is None:
            return

        tx, ty, yaw = self._tf_to_2d(tf)

        points = []
        for det in msg.detections:
            x, y = self._apply_transform_2d(det.position.x, det.position.y, tx, ty, yaw)
            points.append((x, y))

        now = self.get_clock().now().nanoseconds * 1e-9
        self._radar_buffer.append((now, points))
        self._prune_buffer(now)

    # ------------------------------------------------------------------
    # Lidar callback
    # ------------------------------------------------------------------

    def _lidar_cb(self, msg: PoseArray):
        """Keep only lidar people that have a nearby radar detection."""
        now = self.get_clock().now().nanoseconds * 1e-9
        self._prune_buffer(now)

        # Flatten all buffered radar points into one list.
        radar_points = [pt for _, pts in self._radar_buffer for pt in pts]

        if not radar_points:
            # No radar data at all – publish nothing so false positives are suppressed.
            self._publish([], msg)
            self.get_logger().debug("No recent radar data; suppressing all lidar detections.")
            return

        lidar_frame = msg.header.frame_id

        # Transform lidar poses into target_frame for comparison.
        # (If already in target_frame the identity transform is returned.)
        if lidar_frame != self._target_frame:
            tf = self._get_transform(lidar_frame, msg.header.stamp)
            if tf is None:
                return
            tx, ty, yaw = self._tf_to_2d(tf)
        else:
            tx, ty, yaw = 0.0, 0.0, 0.0

        confirmed = []
        for pose in msg.poses:
            wx, wy = self._apply_transform_2d(
                pose.position.x, pose.position.y, tx, ty, yaw
            )
            if self._any_radar_nearby(wx, wy, radar_points):
                confirmed.append(pose)

        self._publish(confirmed, msg)
        self.get_logger().debug(
            f"Fusion: {len(msg.poses)} lidar candidates → {len(confirmed)} confirmed"
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _prune_buffer(self, now_sec: float):
        cutoff = now_sec - self._radar_timeout
        while self._radar_buffer and self._radar_buffer[0][0] < cutoff:
            self._radar_buffer.popleft()

    def _any_radar_nearby(self, px: float, py: float, radar_points: list) -> bool:
        for rx, ry in radar_points:
            if math.hypot(px - rx, py - ry) <= self._threshold:
                return True
        return False

    def _get_transform(self, source_frame: str, stamp):
        """Return the TF from source_frame → target_frame, or None on failure."""
        try:
            return self._tf_buffer.lookup_transform(
                self._target_frame, source_frame, stamp
            )
        except TransformException:
            pass
        try:
            # Fall back to latest known transform (handles small timing gaps).
            return self._tf_buffer.lookup_transform(
                self._target_frame, source_frame, Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"TF {source_frame} → {self._target_frame} unavailable: {ex}",
                throttle_duration_sec=2.0,
            )
            return None

    @staticmethod
    def _tf_to_2d(tf):
        """Extract (tx, ty, yaw) from a TransformStamped."""
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return tx, ty, yaw

    @staticmethod
    def _apply_transform_2d(x: float, y: float, tx: float, ty: float, yaw: float):
        """Apply a 2-D rigid transform (rotation then translation)."""
        wx = math.cos(yaw) * x - math.sin(yaw) * y + tx
        wy = math.sin(yaw) * x + math.cos(yaw) * y + ty
        return wx, wy

    def _publish(self, confirmed_poses: list, source_msg: PoseArray):
        out = PoseArray()
        out.header = source_msg.header
        out.poses = confirmed_poses
        self._people_pub.publish(out)
        self._publish_markers(confirmed_poses, source_msg.header)

    def _publish_markers(self, poses: list, header):
        markers = MarkerArray()

        # Clear stale markers from previous cycle.
        clear = Marker()
        clear.header = header
        clear.ns = "fusion_people"
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, pose in enumerate(poses):
            m = Marker()
            m.header = header
            m.ns = "fusion_people"
            m.id = i + 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD

            m.pose.position.x = pose.position.x
            m.pose.position.y = pose.position.y
            m.pose.position.z = 0.5  # visual midpoint at 0.5 m height
            m.pose.orientation = pose.orientation

            m.scale.x = 0.4
            m.scale.y = 0.4
            m.scale.z = 1.0

            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.8

            m.lifetime.sec = 1  # auto-expire after 1 s if not refreshed

            markers.markers.append(m)

        self._marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
