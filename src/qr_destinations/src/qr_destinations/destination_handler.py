import qr_destinations.lidar_project_to_image as lpi


import cv2

from dataclasses import dataclass


@dataclass
class Destination:
    index: int = -1

    name: str = ""
    tag_id: str = ""



class DestinationHandler(object):
    def __init__(self, 
                 node,
                 aruco_dict=cv2.aruco.DICT_4X4_50):

        self._node = node

        tracked_destinations = []



        # tag stuff
        self._aruco_dict_id = aruco_dict
        self._aruco_dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict)
        self._aruco_params = cv2.aruco.DetectorParameters_create()

    def find_tags(self, lidar_pts, img, pose):

        lidar_project = lpi.LidarProject(img, lidar_pts)
        undistorted_img = lidar_project.img
        img_frame_pts = lidar_project.img_pts
        lidar_frame_pts = lidar_project.lidar_pts

        img_grey = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = cv2.aruco.detectMarkers(
            img_grey, self._aruco_dictionary, parameters=self._aruco_params
        )

        if ids is None:
            print("No tags found in frame")
            return
        
        clean_ids = ids.flatten().tolist()
        
        self._node.get_logger().info(f"Found {[i for i in clean_ids]}")