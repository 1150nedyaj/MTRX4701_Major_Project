from dataclasses import dataclass
from collections import deque
import math
import numpy as np


from radar_messages.msg import StampedReport



@dataclass 
class RD03DMessage:
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
    def __init__(self, rd_x_mm, rd_y_mm, rd_speed_cm):
        self.x = float(rd_y_mm)/1000.0
        self.y = float(rd_x_mm)/1000.0
        self.speed = float(rd_speed_cm)/100.0

        self.r = math.degrees(math.atan2(self.x, self.y))
        self.theta = abs(math.dist((0,0), (self.x, self.y)))
        
    @property
    def covariance(self):
        r_max = 8
        sigma_theta_max = 1.0472

        r_min = 0.1
        sigma_theta_min = 0.0349

        m = (sigma_theta_max - sigma_theta_min) / (r_max - r_min)
        c = sigma_theta_max - (sigma_theta_max - sigma_theta_min) * (r_max / (r_max - r_min))

        sigma_r = 0.3
        sigma_theta = m*self.r + c

        P_polar= np.array([
            [sigma_r**2,0],
            [0,sigma_theta**2]
        ])

        J = np.array([
            [np.sin(self.theta), np.cos(self.theta) * self.r],
            [np.cos(self.theta), np.sin(self.theta) * -self.r]
        ])

        P_xy = J @ P_polar @ J.T
        return P_xy

    @classmethod
    def from_RD03DMessage(cls, rd):
        rd_x_mm = rd.x
        rd_y_mm = rd.y
        rd_speed_cm = rd.speed
        return cls(rd_x_mm, rd_y_mm, rd_speed_cm)

    @property
    def range

class SixteenGateReport:
    gates: list
    
    def __init__(self, gate_values):
        assert len(gate_values) == 16
        self.gates = gate_values

    @classmethod
    def from_stamped_report(cls, msg: StampedReport):
        gates = msg.gate_energies
        return cls(gates)


class gateReportQueue:
    stored: deque
    blank_report: list

    def __init__(self, max_window):
        self.stored = deque(maxlen=max_window)
        self.blank_report = [0] * 16
    
    def push(self, new_report: SixteenGateReport):
        self.stored.append(new_report)

    def sum(self):
        summed_vals = SixteenGateReport(self.blank_report)

        for gR in self.stored:
            for i, g in enumerate(gR.gates):
                summed_vals.gates[i] += g

        return summed_vals

    def avg(self):
        avg_vals = SixteenGateReport(self.blank_report)
        avg_vals.gates =  [(g / len(self.stored))for g in self.sum().gates]

        return avg_vals
        


