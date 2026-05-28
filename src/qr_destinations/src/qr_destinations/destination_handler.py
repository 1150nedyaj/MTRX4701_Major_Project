import cv2
import numpy as np
from math import dist
import matplotlib.pyplot as plt
from copy import deepcopy
import sys

import qr_destinations.lidar_project_to_image as lpi
from qr_destinations.types import Destination, TagDetection
from destination_msgs.msg import DestinationMsg, DestinationListMsg

class DestinationHandler(object):
    def __init__(self, 
                 node,
                 destination_publiser,
                 aruco_dict=cv2.aruco.DICT_4X4_50):

        self._node = node
        self._destinations_publisher = destination_publiser
        self.tracked_destinations = []

        # tag stuff
        self.tag_rad = 0.3              # min space between tags in m
        self.valid_tags = [10,11,12,13,14,15]
        self._aruco_dict_id = aruco_dict
        self._aruco_dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict)
        self._aruco_params = cv2.aruco.DetectorParameters_create()
        

        # plotting (scheming even)
        # plt.ion()
        # self._fig, self._ax_map = plt.subplots(1, 1, figsize=(7, 7))

    def find_tags(self, lidar_pts, img, tf):
        # self._node.get_logger().info(f"find_tags: pts={lidar_pts.shape}, img={img.shape}, dtype={img.dtype}")


        # bring lidar points into camera frame
        lidar_project = lpi.LidarProject(img, lidar_pts)
        # print("STAGE 9: LidarProject built", flush=True, file=sys.stderr)

        undistorted_img = lidar_project.img
        img_frame_pts = lidar_project.img_pts
        lidar_frame_pts = lidar_project.lidar_pts   

        # Extract AruCo tags
        img_grey = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2GRAY)
        # print("STAGE 10: cvtColor done", flush=True, file=sys.stderr)
        corners_list, ids, _ = cv2.aruco.detectMarkers(
            img_grey, self._aruco_dictionary, parameters=self._aruco_params
        )
        # print("STAGE 11: detectMarkers done", flush=True, file=sys.stderr)
        if ids is None:
            # print("No tags found in frame")
            return
        self._node.get_logger().info(f"Found {[i for i in ids.flatten().tolist()]}")

        # order points by bearing
        lidar_xy_full = lidar_frame_pts[:, :2]
        bearings_full = np.arctan2(lidar_xy_full[:, 1], lidar_xy_full[:, 0])
        sort_order = np.argsort(bearings_full)
        sorted_xy = lidar_xy_full[sort_order]
        sorted_img_pts = img_frame_pts[sort_order]

        # associate points that fall within the edges of a tag to it
        detections = []
        for tag_corners, tag_id in zip(corners_list, ids):
            corners = tag_corners.reshape(-1,2)
            u_min = float(corners[:,0].min())
            u_max = float(corners[:,0].max())
            u_coords = sorted_img_pts[:, 0]
            tag_mask = (u_coords >= u_min) & (u_coords <= u_max)   # maybe do expansion along line? 
            tag_bounded_points = sorted_xy[tag_mask]

            if tag_id not in self.valid_tags:
                self._node.get_logger().warn(f"Detected non-tracked tag -> {tag_id}")
                continue        

            if len(tag_bounded_points) == 0:
                self._node.get_logger().warn(f"Hallucinated Tag {tag_id}; no points were found")
                continue

            # current method for normal vector assumes 1+ lidar pts bounded
            d = TagDetection(tag_id, tag_mask, tag_bounded_points)
            d.normal_vector = self._build_normal_vector_from_pts(tag_bounded_points)
            detections.append(d)

        # sorting...
        updates_made = False
        current_destination_tags = [d.tag_id for d in self.tracked_destinations]
        new_detections = [d for d in detections if d.tag_id not in current_destination_tags]
        existing_detections = [d for d in detections if d.tag_id in current_destination_tags]

        # add any previously undetected tags into the tracked list
        for d in new_detections:
            mean_map_coords, _, _ = self.Relative2AbsoluteXY(tf, d.centre)
            mean_map_coords =  mean_map_coords.flatten()
            v_normal_from_map = self.rotate_v_to_map_frame(tf, d.normal_vector)

            if d.normal_vector is None:
                self._node.get_logger().warn(f"Tag {tag_id} has insufficient lidar points, skipping")
                continue

            self._node.get_logger().info(f"detection built from {len(d.bounded_lidar_points)} points...")

            # make sure its not a dodgy reading of the tag
            is_fp = False
            for dest in self.tracked_destinations:
                if dist(dest.tag_centre, mean_map_coords) < self.tag_rad:
                    is_fp = True
                    break
            if is_fp:
                self._node.get_logger().warn(f"Rejecting dodgy reading of {d.tag_id}")
                continue

            new_destination = Destination(
                tag_id=d.tag_id,
                centre=mean_map_coords,
                v_normal=v_normal_from_map,
                pts_used=len(d.bounded_lidar_points)
            )
            self.tracked_destinations.append(deepcopy(new_destination))
            updates_made = True

        # update exisitng detection if this observation uses more points
        for d in existing_detections:
            current_entry = [tD for tD in self.tracked_destinations if tD.tag_id == d.tag_id][0]

            if current_entry.pts_used >= len(d.bounded_lidar_points):
                continue

            self._node.get_logger().info(f"detection built from {len(d.bounded_lidar_points)} points...")    
            
            mean_map_coords, _, _ = self.Relative2AbsoluteXY(tf, d.centre)
            mean_map_coords =  mean_map_coords.flatten()
            v_normal_from_map = self.rotate_v_to_map_frame(tf, d.normal_vector)

            current_entry.tag_centre = mean_map_coords
            current_entry.wall_normal_vector = v_normal_from_map
            current_entry.pts_used = len(d.bounded_lidar_points)

            updates_made = True

        if updates_made:
            self._publish_tracked()
            # self._update_map_plot(tf) 

    def _publish_tracked(self):
        dest_list = DestinationListMsg()

        for d in self.tracked_destinations:
            d_msg = d.to_DestinationMsg()
            dest_list.destinations.append(d_msg)

        self._destinations_publisher.publish(dest_list)

    def _update_map_plot(self, tf):
        self._ax_map.cla()
        self._ax_map.set_title('Map')
        self._ax_map.set_xlabel('x (m)')
        self._ax_map.set_ylabel('y (m)')
        self._ax_map.set_aspect('equal')
        self._ax_map.grid(True)

        # robot position from tf
        rx = tf.translation.x
        ry = tf.translation.y
        ryaw = self.yaw_from_quaternion(tf.rotation)
        self._ax_map.scatter(rx, ry, s=80, c='limegreen', zorder=4)
        self._ax_map.annotate(
            '', xy=(rx + 0.15 * np.cos(ryaw), ry + 0.15 * np.sin(ryaw)), xytext=(rx, ry),
            arrowprops=dict(arrowstyle='->', color='limegreen', lw=1.5)
        )

        for dest in self.tracked_destinations:
            cx, cy = dest.tag_centre[0], dest.tag_centre[1]
            gx, gy = dest.destination_point[0], dest.destination_point[1]

            self._ax_map.scatter(cx, cy, s=60, c='steelblue', zorder=3)
            self._ax_map.scatter(gx, gy, s=60, marker='*', c='gold', zorder=3)
            self._ax_map.annotate(
                '', xy=(gx, gy), xytext=(cx, cy),
                arrowprops=dict(arrowstyle='->', color='tomato', lw=1.5)
            )
            self._ax_map.text(gx + 0.02, gy + 0.02, f'tag {int(dest.tag_id)}', fontsize=8)

        self._fig.canvas.draw()
        self._fig.canvas.flush_events()
        
    def rotate_v_to_map_frame(self, abs_tf, vector):
        """
        Rotate vector so it can be refered to in map frame
        """
        theta = self.yaw_from_quaternion(abs_tf.rotation)
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])

        return R @ vector
    
    def Relative2AbsoluteXY(self, abs_tf, landmark_position_rel):
        """
        Adapted from: ACFR-RPG/ekf-landmark-slam -> Arihant Lunawat

        Convert's a landmark's position from the robot's frame of reference to the absolute frame of reference
        :param abs_tf: transform of the robot from the map frame (absolute)
        :param landmark_position_rel: position of the landmark in the robot's frame of reference [x, y]
        :return : [position of the landmark in the absolute frame of reference [x, y], G1, G2]
        """

        x1 = abs_tf.translation.x
        y1 = abs_tf.translation.y
        theta1 = self.yaw_from_quaternion(abs_tf.rotation)
        
        x2 = landmark_position_rel[0]
        y2 = landmark_position_rel[1]

        landmark_position_rel_vec = np.array([[x2], [y2], [1]])

        # R is the transition matrix to robot frame
        R = np.array(
            [
                [np.cos(theta1), -np.sin(theta1), 0],
                [np.sin(theta1), np.cos(theta1), 0],
                [0, 0, 1],
            ]
        )

        # Calculate Jacobian H1 with respect to X1
        G1 = np.array(
            [
                [1, 0, -x2 * np.sin(theta1) - y2 * np.cos(theta1)],
                [0, 1, x2 * np.cos(theta1) - y2 * np.sin(theta1)],
            ]
        )

        # Calculate Jacobian H2 with respect to X2
        G2 = np.array([[np.cos(theta1), -np.sin(theta1)], [np.sin(theta1), np.cos(theta1)]])

        landmark_abs = np.array(np.dot(R, landmark_position_rel_vec)) + np.array(
            [[x1],[y1],[theta1]]
        )

        return np.array([[landmark_abs[0][0]], [landmark_abs[1][0]]]), G1, G2
    
    @staticmethod
    def _build_normal_vector_from_pts(pts):
        """
        Gives you a normal vector for a line of points,
        making sure it doesn't point into objects that its detecting.
        """

        m, c = np.polyfit(pts[:,0], pts[:,1], 1)

        v_normal = np.array([-m, 1.0])
        v_normal = v_normal / np.linalg.norm(v_normal)

        # flip v_normal if pointing away from origin
        v_mean_to_origin = -pts.mean(axis=0)
        if np.dot(v_normal, v_mean_to_origin) < 0:
            v_normal = -v_normal

        return v_normal



    @staticmethod
    def yaw_from_quaternion(q) -> float:
        """Extract yaw (rotation about Z) from a geometry_msgs Quaternion."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return float(np.arctan2(siny_cosp, cosy_cosp))