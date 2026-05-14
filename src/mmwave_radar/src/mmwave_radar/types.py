from dataclasses import dataclass
from collections import deque

from radar_messages.msg import StampedReport

@dataclass
class RadarFrame:
    present: bool
    distance: int
    gate_energies: tuple

    def __str__(self):
        return f"Distance: {self.distance}, Gates: {self.gate_energies}"
    
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

