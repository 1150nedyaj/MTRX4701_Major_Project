#!/usr/bin/env python3
"""
MTRX4701 2026 Assignment 4
File: sensor_fusion_node.py
Author(s): Jeremy Fox, Adrian Le [LiDAR Masking]

Fuses mmWave Radar Detections with LiDAR Ankle detections, using the 
mahalanobis distances. This was attempted with euclidan distance, but 
association values that worked well in front of the robot, were leading
to incorrect fusion's behind the bot. 

Displays a live plot via. the signature_plotter, which was used during 
tuning for the system.

Uses the custom SignatureQueue's to hold all detections and people currently 
being tracked.

Radar Detections in a SignatureQueue were tending to overlap a lot, so opted
to match LiDAR Detections to Radar ones. Taking the closest one and treating it
according to the distance.

Once it gets a match, the program moves on to compare it with the people that it's
currently tracking. Tracked humans hang around for a longer period, so as to give 
Nav2 a more consistent set of points over time. Without this tracking, people detections
were flickering in and out of the system, and Nav2 wasn't able to properly develop a costmap 
from the points it was being provided.

Right now it's only tracking one person reliably. I haven't had a chance to check if
that's purely a sensor problem, or if this fusion logic needs another look.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point

from geometry_msgs.msg import PoseArray, PointStamped, Pose
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan

from radar_messages.msg import StampedRadarDetections
from sensor_fusion.types import SignatureQueue, RadarPersonSignature, LidarAnkleSignature, Signature
from sensor_fusion.signature_plotter import SignaturePlotter


class SensorFusionNode(Node):

    def __init__(self):
        super().__init__("sensor_fusion")

        self.exclusion_rad = 0.3

        # --- Signature Queue Parameters ---
        radar_stale_ms = 500
        lidar_stale_ms = 100            # make smaller?
        fusion_stale_ms = 1000

        self.fusion_assoc_mahal = 2.5   # ~95% certainty for 2 dof system
        self.human_assoc_mahal = 2.5    # ~95% certainty for 2 dof system
        self.new_human_mahal = 15       # 4 std. dev. --> ~100% certainity for 2 dof system

        # --- ROS parameters ---
        self.declare_parameter("radar_topics", ["/mmWave_array/radar_0/detections"])
        self.declare_parameter("fusion_distance_threshold", 1.0)
        self.declare_parameter("radar_timeout", 1.5)
        self.declare_parameter("hold_time", 1.0)
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("human_extraction_radius", 0.45)

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
        self._extraction_radius = (
            self.get_parameter("human_extraction_radius")
            .get_parameter_value()
            .double_value
        )


        # --- TF ---
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)


        # --- mmRadar Detection Subscribers (one per topic) ---
        self._radar_subs = []
        for topic in radar_topics:
            sub = self.create_subscription(
                StampedRadarDetections, topic, self._radar_detection_callback, 10
            )
            self._radar_subs.append(sub)
            self.get_logger().info(f"Subscribed to radar: {topic}")


        # --- LiDAR Related Subscriptions --- 
        self._ankle_sub = self.create_subscription(
            MarkerArray, 
            "/lidar/circle_markers", 
            self._ankle_marker_callback, 
            10
        )

        self._lidar_scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            10,
        )


        # --- Publishers ---
        self._people_pub = self.create_publisher(PoseArray, "/fusion/people", 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, "/fusion/people_markers", 10
        )
        self._radar_marker_pub = self.create_publisher(
            MarkerArray, "/fusion/radar_markers", 10
        )
        self._people_scan_pub = self.create_publisher(LaserScan, "/fusion/human_scan", 10)


        # --- Signature Queues --- 
        self._plotter = SignaturePlotter()
        self.radar_queue = SignatureQueue('radar_people', radar_stale_ms)
        self.ankle_queue = SignatureQueue('lidar_people', lidar_stale_ms)
        self.people_queue = SignatureQueue('fusion_people', fusion_stale_ms)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _radar_detection_callback(self, msg: StampedRadarDetections):
        """
        Adds to the tracked radar signatures
        """

        # Parse the radar data and add the new radar detections to the radar queue
        if not msg.detections:
            return

        source_frame = msg.header.frame_id
        tf = self._get_transform(source_frame, msg.header.stamp)
        if tf is None:
            return

        t_ms = self.millis_from_rclpy_time(self.get_clock().now())
        c_det = 0
        for det in msg.detections:
            z = getattr(det.position, 'z', 0.0)
            wx, wy, wz = self._transform_point_3d(
                det.position.x, det.position.y, z, tf
            )

            s = RadarPersonSignature(wx, wy, det.covariance,t_ms)
            self.radar_queue.add(s)
            c_det += 1

        # clear out the stale detections from radar queue
        self.radar_queue.clean_out(t_ms)

        self.get_logger().info(f"Radar Queue - Added {c_det} - Size {self.radar_queue.size}")

    def _ankle_marker_callback(self, msg):
        """
        Adds to the tracked ankle signatures
        """

        self.get_logger().info('Marker fired')
        if not msg.markers:
            self.get_logger().error("Message empty")
            return

        # Add new Ankles to Queue
        t_ms = self.millis_from_rclpy_time(self.get_clock().now())
        c_det = 0
        for m in msg.markers:
            # the detector publishes positions in points[], not pose.position
            if not m.points:
                continue

            # optional: only take the paired "Person Centers", skip raw ankles
            # if m.ns != "Person Centers":
            #     continue

            lidar_frame = m.header.frame_id
            if lidar_frame and lidar_frame != self._target_frame:
                tf = self._get_transform(lidar_frame, m.header.stamp)
                if tf is None:
                    continue
                tx, ty, yaw = self._tf_to_2d(tf)
            else:
                tx, ty, yaw = 0.0, 0.0, 0.0

            for pt in m.points:
                wx, wy = self._apply_transform_2d(pt.x, pt.y, tx, ty, yaw)
                self.ankle_queue.add(LidarAnkleSignature(wx, wy, t_ms))
                c_det += 1

        # clear out stale detections from ankle queue
        self.ankle_queue.clean_out(t_ms)
        self.get_logger().info(f"Ankle Queue - Added {c_det} - Size {self.ankle_queue.size}")

    def _scan_callback(self, msg: LaserScan):
        """
        Fuse the signatures, then use them to make a mask over the LiDAR scan 
        """

        # get tracked people up to date
        self.run_ankle_radar_fusion(self.ankle_queue,
                                    self.radar_queue,
                                    self.people_queue)
        
        # Handle fallback safely if tracker states are unpopulated
        if not self.people_queue.values:
            empty_scan = self._create_empty_scan_layout(msg)
            self._people_scan_pub.publish(empty_scan)
            self._plotter.update_plots([self.radar_queue, self.ankle_queue, self.people_queue])
            self._publish_poses_from_SignatureQueue(self.people_queue)
            return

        # Compute transformation to unify target frames
        scan_frame = msg.header.frame_id
        if scan_frame != self._target_frame:
            tf = self._get_transform(scan_frame, msg.header.stamp)
            if tf is None:
                return
            tx, ty, yaw = self._tf_to_2d(tf)
        else:
            tx, ty, yaw = 0.0, 0.0, 0.0

        # Mask index values mapping inside tracking thresholds
        masked_ranges = [float('inf')] * len(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        for i, dist in enumerate(msg.ranges):
            if not np.isfinite(dist) or dist < msg.range_min or dist > msg.range_max:
                continue

            # Project into local 2D space
            angle = angle_min + (i * angle_increment)
            lx = dist * math.cos(angle)
            ly = dist * math.sin(angle)

            # Re-project to core tracking frame (e.g. base_link)
            wx, wy = self._apply_transform_2d(lx, ly, tx, ty, yaw)

            # Match directly against the values inside your core signature tracker queue
            for person in self.people_queue.values:
                if math.hypot(wx - person.x, wy - person.y) <= self._extraction_radius:
                    masked_ranges[i] = dist
                    break

        # Construct output scan message and publish
        output_scan = self._create_empty_scan_layout(msg)
        output_scan.header.stamp = self.get_clock().now().to_msg()
        output_scan.ranges = masked_ranges
        self._people_scan_pub.publish(output_scan)

        # Standard visualization updates
        self._plotter.update_plots([self.radar_queue, self.ankle_queue, self.people_queue])
        self._publish_poses_from_SignatureQueue(self.people_queue)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _create_empty_scan_layout(self, source_msg: LaserScan) -> LaserScan:
        """
        Clones message descriptor topologies containing static laser definitions.
        """
        scan = LaserScan()
        scan.header = source_msg.header
        scan.angle_min = source_msg.angle_min
        scan.angle_max = source_msg.angle_max
        scan.angle_increment = source_msg.angle_increment
        scan.time_increment = source_msg.time_increment
        scan.scan_time = source_msg.scan_time
        scan.range_min = source_msg.range_min
        scan.range_max = source_msg.range_max
        scan.ranges = [float('inf')] * len(source_msg.ranges)
        if source_msg.intensities:
            scan.intensities = source_msg.intensities
        return scan
    
    def run_ankle_radar_fusion(self, ankle_queue: SignatureQueue, radar_queue: SignatureQueue, fusion_queue: SignatureQueue):
        """
        Try matching each LiDAR to Radar, if they're close it's a person candidate.
        If a person candidate is close to an existing person, update the info.
        If not, add a new person to the SignatureQueue.
        """

        # clear out the stale people
        t_ms = self.millis_from_rclpy_time(self.get_clock().now())
        self.people_queue.clean_out(t_ms)

        # match ankles to radar
        fused = []
        for a in ankle_queue.values:
            nearest = self.nearest_signature(a, radar_queue.values)
            if nearest is None:
                continue

            r, dist = nearest
            if dist <= self.fusion_assoc_mahal:
                fused.append(a+r)
    

        # try to match up fused with people
        for f in fused:
            nearest = self.nearest_signature(f, fusion_queue.values)

            if nearest is not None and nearest[1] <= self.human_assoc_mahal:
                tracked = nearest[0]
                tracked.x = f.x
                tracked.y = f.y
                tracked.covariance = f.covariance
                tracked.t = f.t

            elif nearest is None or nearest[1] > self.new_human_mahal:
                fusion_queue.add(f)

    @staticmethod
    def nearest_signature(query:Signature, candidates):
        """
        With a given query signature, find me the closest one from
        a list of candidate signatures
        """

        best_sig = None
        best_dist = float('inf')

        for c in candidates:
            d = SensorFusionNode.mahalanobis_distance(query, c)
            if d < best_dist:
                best_dist = d
                best_sig = c

        if best_sig is None:
            return None
        
        return best_sig, best_dist

    @staticmethod
    def mahalanobis_distance(sig_a: Signature, sig_b:Signature):
        """
        Mahalanobis distance between two signatures, accounting for the
        uncertainty (covariance) of both.
        """

        dx = sig_a.x - sig_b.x
        dy = sig_a.y - sig_b.y
        delta = np.array([dx, dy])

        cov_a = np.array(sig_a.covariance, dtype=float).reshape(2, 2)
        cov_b = np.array(sig_b.covariance, dtype=float).reshape(2, 2)
        combined = cov_a + cov_b

        try:
            inv = np.linalg.inv(combined)
        except np.linalg.LinAlgError:
            inv = np.linalg.pinv(combined)

        d_sq = float(delta.T @ inv @ delta)
        return math.sqrt(max(d_sq, 0.0))

    @staticmethod
    def millis_from_rclpy_time(rclpy_time):
        """
        Converts the ros node time into something that's
        conveinient for timing.
        """

        s_ns = rclpy_time.seconds_nanoseconds()
        return s_ns[0] * 1000.0 + s_ns[1] * 1e-6

    def _get_transform(self, source_frame: str, stamp):
        """
        Return the TF from source_frame → target_frame, or None on failure.
        """

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
        """
        Extract (tx, ty, yaw) from a TransformStamped.
        """

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
        """
        Rotate then translate: body-frame point → world-frame point.
        """
        
        wx = math.cos(yaw) * x - math.sin(yaw) * y + tx
        wy = math.sin(yaw) * x + math.cos(yaw) * y + ty
        return wx, wy
    
    def _transform_point_3d(self, x, y, z, tf):
        """
        Transform a 3D point through the full TF (incl. roll/pitch/yaw), return (x, y, z) in target_frame.
        """

        p = PointStamped()
        p.point.x = float(x)
        p.point.y = float(y)
        p.point.z = float(z)

        out = do_transform_point(p, tf)
        return out.point.x, out.point.y, out.point.z

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_poses_from_SignatureQueue(self, sig_queue: SignatureQueue):
        """
        Get a signature queue up onto the human topic,
        only intended for fused human signatures.
        """

        pose_array = PoseArray()
        pose_array.header.frame_id = self._target_frame
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for s in sig_queue.values:
            p = Pose()
            p.position.x = s.x
            p.position.y = s.y
            p.position.z = 0.0
            p.orientation.w = 1.0
            pose_array.poses.append(p)

        self._people_pub.publish(pose_array)
    
def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
