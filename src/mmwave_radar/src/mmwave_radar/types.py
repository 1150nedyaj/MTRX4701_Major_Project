from dataclasses import dataclass

@dataclass
class RadarFrame:
    present: bool
    distance: int
    gate_energies: tuple

    def __str__(self):
        return f"Distance: {self.distance}, Gates: {self.gate_energies}"