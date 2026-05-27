from dataclasses import dataclass
import numpy as np
from copy import deepcopy

from destination_msgs.msg import DestinationMsg
from geometry_msgs.msg import Pose

@dataclass
class Destination:
    tag_id: int
    tag_centre: np.ndarray
    wall_normal_vector: np.array
    pts_used: int

    name: str = ""

    _wall_offset: float = 0.1

    def __init__(self, tag_id, centre, v_normal, pts_used):
        self.tag_id = tag_id
        self.tag_centre = centre
        self.wall_normal_vector = v_normal
        self.pts_used = pts_used

    def to_DestinationMsg(self):
        d_msg = DestinationMsg()

        d_msg.name = self.name
        d_msg.tag = int(self.tag_id)

        goal_pose_msg = Pose()

        p_dest = self.destination_point
        goal_pose_msg.position.x = float(p_dest[0])
        goal_pose_msg.position.y = float(p_dest[1])
        goal_pose_msg.position.z = 0.0

        yaw = np.arctan2(self.wall_normal_vector[1], self.wall_normal_vector[0])
        goal_pose_msg.orientation.x = 0.0
        goal_pose_msg.orientation.y = 0.0
        goal_pose_msg.orientation.z = np.sin(yaw / 2.0)
        goal_pose_msg.orientation.w = np.cos(yaw / 2.0)

        d_msg.pose = goal_pose_msg

        return deepcopy(d_msg)



    @property
    def destination_point(self):
        return self.tag_centre + self._wall_offset * self.wall_normal_vector


@dataclass
class TagDetection:
    tag_id: int 
    
    bounded_lidar_points: np.ndarray 
    lidar_point_mask: np.ndarray 

    normal_vector: np.array

    def __init__(self, tag_id, pts_mask, bounded_pts):
        self.tag_id = tag_id
        self.bounded_lidar_points = bounded_pts
        self.lidar_point_mask = pts_mask
    
    @property
    def centre(self):
        return np.mean(self.bounded_lidar_points, axis=0) 