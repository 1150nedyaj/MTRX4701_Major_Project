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
        


