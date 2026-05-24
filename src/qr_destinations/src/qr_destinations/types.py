from dataclasses import dataclass
import numpy as np

@dataclass
class Destination:
    tag_id: int
    tag_centre: np.ndarray
    
    wall_normal_vector: np.array

    name: str = ""

    _wall_offset: float = 0.1

    def __init__(self, tag_id, centre, v_normal):
        self.tag_id = tag_id
        self.tag_centre = centre
        self.wall_normal_vector = v_normal

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