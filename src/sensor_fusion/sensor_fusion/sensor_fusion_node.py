#!/usr/bin/env python3
"""
Sensor fusion: confirm lidar people detections with radar.

A lidar-detected person is forwarded only when a radar detection exists
within a configurable spatial and temporal window.  This eliminates false
positives that arise from circular static objects (traffic cones, poles,
wheels) which the circle-detector cannot distinguish from legs.

Once a person is confirmed by both sensors they are added to a tracked-people
buffer.  The track is kept alive by radar alone — if radar continues to detect
something near the tracked position, the hold timer resets even when the lidar
circle-detector misses that person's legs for several frames.  The track
expires when neither sensor has refreshed it within `hold_time` seconds.

Output is published by a dedicated 10 Hz timer so that confirmed-person
markers remain visible in RViz even when the lidar scan drops out entirely
for one or more frames.

Subscribes
----------
/lidar/circle_candidates          (geometry_msgs/PoseArray)
    People positions from the lidar circle-detector, in the scan frame.
<radar_topic>                     (radar_messages/StampedRadarDetections)
    Raw radar detections, in each module's own TF frame.
    Default topic: /mmWave_array/radar_0/detections
    Override via the "radar_topics" parameter (string array).

Publishes
---------
/fusion/people                    (geometry_msgs/PoseArray)
    Radar-confirmed people (with hold), same frame as lidar input.
/fusion/people_markers            (visualization_msgs/MarkerArray)
    Green cylinders for confirmed/held people (RViz).
/fusion/radar_markers             (visualization_msgs/MarkerArray)
    Orange spheres showing every buffered radar detection in target_frame
    (useful for tuning and visualising radar coverage in RViz).

Parameters
----------
radar_topics              string[]  – radar detection topic names
fusion_distance_threshold double    – max distance (m) for a radar point to
                                      confirm a lidar detection  [default 1.0]
radar_timeout             double    – keep radar readings for this many seconds
                                      [default 1.5]
hold_time                 double    – keep a confirmed track alive for this many
                                      seconds after its last confirmation
                                      [default 1.0]
target_frame              string    – common TF frame for spatial comparison
                                      [default "base_link"]
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseArray
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import Buffer, TransformListener, TransformException

from radar_messages.msg import StampedRadarDetections


class SensorFusionNode(Node):

    def __init__(self):
        super().__init__("sensor_fusion")

        # --- parameters ---
        self.declare_parameter("radar_topics", ["/mmWave_array/radar_0/detections"])
        self.declare_parameter("fusion_distance_threshold", 1.0)
        self.declare_parameter("radar_timeout", 1.5)
        self.declare_parameter("hold_time", 1.0)
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
        self._hold_time = (
            self.get_parameter("hold_time").get_parameter_value().double_value
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

        # --- tracked people: list of dicts ---
        # Each entry: {wx, wy, pose, last_confirmed}
        #   wx/wy          – position in target_frame, kept up to date for matching
        #   pose           – most recent Pose in the lidar frame, used for output
        #   last_confirmed – wall time (sec) of the most recent radar confirmation
        self._tracked_people: list = []

        # Frame id of the last received lidar message, used by the publish timer
        # to stamp outgoing PoseArray messages with the correct frame.
        self._lidar_frame: str = "base_scan"

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
        self._radar_marker_pub = self.create_publisher(
            MarkerArray, "/fusion/radar_markers", 10
        )

        # --- publish timer ---
        # Drives expiry and output at 10 Hz, independent of the lidar scan rate.
        # This keeps RViz markers alive even when the lidar scan drops out for
        # one or more frames — previously the markers would auto-expire because
        # the output was only published inside _lidar_cb.
        self._publish_timer = self.create_timer(0.1, self._publish_timer_cb)

        self.get_logger().info(
            f"sensor_fusion ready | threshold={self._threshold} m | "
            f"radar_timeout={self._radar_timeout} s | "
            f"hold_time={self._hold_time} s | "
            f"frame={self._target_frame}"
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
            x, y = self._apply_transform_2d(
                det.position.x, det.position.y, tx, ty, yaw
            )
            points.append((x, y))

        now = self.get_clock().now().nanoseconds * 1e-9
        self._radar_buffer.append((now, points))
        self._prune_radar_buffer(now)

        # Keep confirmed tracks alive during lidar dropouts.
        # If radar sees something near a tracked person, refresh the hold timer
        # without waiting for a lidar candidate to co-confirm.  This prevents
        # tracks from expiring when the circle-detector misses a frame.
        # Radar cannot open new tracks — only lidar+radar agreement does that.
        for person in self._tracked_people:
            if self._any_radar_nearby(person["wx"], person["wy"], points):
                person["last_confirmed"] = now

        # Publish orange sphere markers so radar is visible in RViz.
        self._publish_radar_markers(msg.header)

    # ------------------------------------------------------------------
    # Lidar callback
    # ------------------------------------------------------------------

    def _lidar_cb(self, msg: PoseArray):
        """Confirm new people from the lidar scan and update the tracked list.

        Expiry and publishing are handled by the publish timer so that output
        continues even when the lidar scan drops out for a frame or more.
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        self._prune_radar_buffer(now)

        # Remember the lidar frame so the publish timer can stamp its output.
        self._lidar_frame = msg.header.frame_id

        # Flatten all buffered radar points into one list.
        radar_points = [pt for _, pts in self._radar_buffer for pt in pts]

        # Compute the lidar → target_frame transform once for this scan.
        lidar_frame = msg.header.frame_id
        if lidar_frame != self._target_frame:
            tf = self._get_transform(lidar_frame, msg.header.stamp)
            if tf is None:
                return
            tx, ty, yaw = self._tf_to_2d(tf)
        else:
            tx, ty, yaw = 0.0, 0.0, 0.0

        # --- step 1: find which lidar candidates are radar-confirmed right now ---
        newly_confirmed = []  # (wx, wy, original_pose)
        for pose in msg.poses:
            wx, wy = self._apply_transform_2d(
                pose.position.x, pose.position.y, tx, ty, yaw
            )
            if radar_points and self._any_radar_nearby(wx, wy, radar_points):
                newly_confirmed.append((wx, wy, pose))

        # --- step 2: merge confirmations into the tracked-people list ---
        for wx, wy, pose in newly_confirmed:
            matched = self._find_tracked(wx, wy)
            if matched is not None:
                # Refresh the existing track with the latest position and time.
                matched["wx"] = wx
                matched["wy"] = wy
                matched["pose"] = pose
                matched["last_confirmed"] = now
            else:
                # Brand-new person — open a fresh track.
                self._tracked_people.append(
                    {"wx": wx, "wy": wy, "pose": pose, "last_confirmed": now}
                )

        self.get_logger().debug(
            f"Fusion: {len(msg.poses)} lidar | "
            f"{len(newly_confirmed)} confirmed now | "
            f"{len(self._tracked_people)} tracked"
        )

    # ------------------------------------------------------------------
    # Publish timer
    # ------------------------------------------------------------------

    def _publish_timer_cb(self):
        """Expire stale tracks and publish output at a fixed 10 Hz rate.

        Running this independently of the lidar callback means confirmed-person
        markers stay visible in RViz during lidar scan dropouts.
        """
        now = self.get_clock().now().nanoseconds * 1e-9

        # Expire tracks that have not been refreshed within hold_time.
        self._tracked_people = [
            p for p in self._tracked_people
            if (now - p["last_confirmed"]) < self._hold_time
        ]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._lidar_frame

        self._publish([p["pose"] for p in self._tracked_people], header)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _find_tracked(self, wx: float, wy: float) -> dict | None:
        """Return the closest tracked person within threshold, or None."""
        best, best_dist = None, self._threshold
        for person in self._tracked_people:
            d = math.hypot(wx - person["wx"], wy - person["wy"])
            if d < best_dist:
                best_dist = d
                best = person
        return best

    def _prune_radar_buffer(self, now_sec: float):
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
        """Rotate then translate: body-frame point → world-frame point."""
        wx = math.cos(yaw) * x - math.sin(yaw) * y + tx
        wy = math.sin(yaw) * x + math.cos(yaw) * y + ty
        return wx, wy

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish(self, poses: list, header: Header):
        out = PoseArray()
        out.header = header
        out.poses = poses
        self._people_pub.publish(out)
        self._publish_people_markers(poses, header)

    def _publish_people_markers(self, poses: list, header: Header):
        markers = MarkerArray()

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
            m.pose.position.z = 0.5
            m.pose.orientation = pose.orientation

            m.scale.x = 0.4
            m.scale.y = 0.4
            m.scale.z = 1.0

            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.8

            m.lifetime.sec = int(self._hold_time) + 1

            markers.markers.append(m)

        self._marker_pub.publish(markers)

    def _publish_radar_markers(self, source_header: Header):
        """Publish all buffered radar points as orange spheres in target_frame."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._target_frame

        markers = MarkerArray()

        clear = Marker()
        clear.header = header
        clear.ns = "radar_detections"
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        marker_id = 0
        for _, points in self._radar_buffer:
            for wx, wy in points:
                m = Marker()
                m.header = header
                m.ns = "radar_detections"
                m.id = marker_id
                marker_id += 1

                m.type = Marker.SPHERE
                m.action = Marker.ADD

                m.pose.position.x = wx
                m.pose.position.y = wy
                m.pose.position.z = 0.1  # slightly above ground
                m.pose.orientation.w = 1.0

                m.scale.x = 0.3
                m.scale.y = 0.3
                m.scale.z = 0.3

                # Orange — visually distinct from the green people cylinders.
                m.color.r = 1.0
                m.color.g = 0.5
                m.color.b = 0.0
                m.color.a = 0.9

                # Auto-expire slightly after the buffer timeout so stale
                # markers clean themselves up if the node stops publishing.
                lifetime_sec = self._radar_timeout + 0.5
                m.lifetime.sec = int(lifetime_sec)
                m.lifetime.nanosec = int((lifetime_sec % 1) * 1e9)

                markers.markers.append(m)

        self._radar_marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
