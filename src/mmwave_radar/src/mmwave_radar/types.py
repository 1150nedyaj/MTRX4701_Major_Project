"""
MTRX4701 2026 Assignment 4
File: types.py
Author(s): Jeremy Fox

These are the custom types for the radar package.
RD03DMessage's get built from the radar's output, while the RadarSignature
is in essence just the radar detection message type.
"""

from dataclasses import dataclass
from collections import deque
import math
import numpy as np

@dataclass 
class RD03DMessage:
    """
    Adapted from the setup with the old radar.
    Only encode's range bearing data
    """

    def __init__(self, x, y, speed, pixel_distance, detection):
        self.x = x                  # mm
        self.y = y                  # mm
        self.speed = speed          # cm/s
        self.pixel_distance = pixel_distance  # mm
        self.distance = math.sqrt(x**2 + y**2)
        self.angle = math.degrees(math.atan2(x, y))
        self.detection = detection
    
    def __str__(self):
        return ('Target(x={}mm, y={}mm, speed={}cm/s, pixel_dist={}mm, '
                'distance={:.1f}mm, angle={:.1f}°)').format(
                self.x, self.y, self.speed, self.pixel_distance, self.distance, self.angle)

@dataclass
class RadarSignature:
    """
    The middle-man between straight radar messages and the detction topic.
    Adds covariance to the range bearing detection, such that it reflects
    the characteristics of the radar module. 

    This does assume all the radars being used have the same physical characteristics
    """

    def __init__(self, rd_x_mm, rd_y_mm, rd_speed_cm):
        self.x = float(rd_y_mm)/1000.0
        self.y = float(rd_x_mm)/1000.0
        self.speed = float(rd_speed_cm)/100.0

        self.r = abs(math.dist((0,0), (self.x, self.y)))
        self.theta = math.atan2(self.y, self.x)

        
    def __str__(self):
        return f"({self.x}, {self.y}) : ({self.r}, {np.degrees(self.theta):.2f}°)"

    @property
    def covariance(self):
        """
        Calculates uncertainty in (r, theta) as bigger r -> bigger theta uncertainty.
        Converts to (x,y) as rest of system uses cartesian.
        """

        r_max = 8                   # --> Max. range we'll use this for
        sigma_theta_max = 1.0472    # --> 60 degrees --> 120 degree F.O.V.

        r_min = 0.1                 # --> Closest a person can feasibly get is 0.15m
        sigma_theta_min = 0.1745    # --> 10 degrees --> certainity but no covar. collapse

        m = (sigma_theta_max - sigma_theta_min) / (r_max - r_min)
        c = sigma_theta_max - (sigma_theta_max - sigma_theta_min) * (r_max / (r_max - r_min))

        sigma_r = 0.4
        sigma_theta = m*self.r + c

        P_polar= np.array([
            [sigma_r**2,0],
            [0,sigma_theta**2]
        ])

        J = np.array([
            [np.cos(self.theta), np.sin(self.theta) * -self.r],
            [np.sin(self.theta), np.cos(self.theta) * self.r]
        ])

        P_xy = J @ P_polar @ J.T
        return P_xy

    @classmethod
    def from_RD03DMessage(cls, rd):
        rd_x_mm = rd.x
        rd_y_mm = rd.y
        rd_speed_cm = rd.speed
        return cls(rd_x_mm, rd_y_mm, rd_speed_cm)

