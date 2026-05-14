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
        
    def __str__(self):
        return f"({self.x}, {self.y}) : ({self.r}, {np.degrees(self.theta)}°)"

    @property
    def covariance(self):
        r_max = 8
        sigma_theta_max = 1.0472    # --> 60 degrees

        r_min = 0.1
        sigma_theta_min = 0.0349    # --> 2 degrees

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


class SixteenGateReport:
    gates: list
    
    def __init__(self, gate_values):
        assert len(gate_values) == 16
        self.gates = gate_values

    def __sub__(self, other):
        # no such thing as -ve. intensity
        new_gates = [0] * 16
        for i in range(16):
            new_gates[i] = max(0, self.gates[i] - other.gates[i])
        return SixteenGateReport(new_gates)

    def dist_est(self, gate_size_m=0.625):
        """
        ---
        CLAUDE Sonnet 4.7
        ---

        Estimate the distance of a single target as the energy-weighted
        centroid of the gate intensities.

        Each gate i corresponds to range bin centred at (i + 0.5) * gate_size_m
        metres from the radar. Returns the weighted mean of those bin centres,
        using gate energies as weights.

        Returns None if total energy is zero (no detection / empty report).
        """
        dist_gates = self.gates[2:]

        total_energy = sum(dist_gates)
        if total_energy <= 0:
            return float('nan')

        bin_centres = [(i + 0.5) * gate_size_m for i in range(len(dist_gates))]
        weighted = sum(c * e for c, e in zip(bin_centres, dist_gates))
        return weighted / total_energy

    @classmethod
    def from_stamped_report(cls, msg: StampedReport):
        gates = msg.gate_energies
        return cls(gates)


class GateReportQueue:
    def __init__(self, max_window):
        self.stored = deque(maxlen=max_window)
        self.sensor_name = 'none'

    def set_name(self, name):
        self.sensor_name = name

    @property
    def name(self):
        return self.sensor_name

    def push(self, new_report: SixteenGateReport):
        self.stored.append(new_report)

    @property
    def sum(self):
        totals = [0] * 16 
        for gR in self.stored:
            for i, g in enumerate(gR.gates):
                totals[i] += g
        return SixteenGateReport(totals)

    @property
    def avg(self):
        if not self.stored:
            return SixteenGateReport([0] * 16)
        n = len(self.stored)
        return SixteenGateReport([g / n for g in self.sum.gates])

    @property
    def top(self):
        return self.stored[-1] if self.stored else None

