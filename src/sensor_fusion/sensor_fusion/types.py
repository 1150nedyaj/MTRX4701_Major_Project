from dataclasses import dataclass
import numpy as np
from copy import deepcopy

@dataclass
class Signature:
    def __init__(self, x:float, y:float, flat_covariance, t_birth_ms:float):
        self.x = x
        self.y = y
        
        self.covariance = flat_covariance

        self.t = t_birth_ms

    def age(self, t_ms):
        return t_ms - self.t

    def __add__(self, other):
        avg_x = (self.x + other.x) / 2.0
        avg_y = (self.y + other.y) / 2.0

        cov_a = np.array(self.covariance, dtype=float)
        cov_b = np.array(other.covariance, dtype=float)
        avg_cov = ((cov_a + cov_b) / 2.0).tolist()

        t_birth = max(self.t, other.t)

        return Signature(avg_x, avg_y, avg_cov, t_birth)
    
@dataclass
class RadarPersonSignature(Signature):
    def __init__(self, x, y, flat_covariance, t):
        super().__init__(x, y, flat_covariance, t)

@dataclass
class LidarAnkleSignature(Signature):
    def __init__(self, x, y, t):
        PERSON_RAD = 0.15

        flat_covar = [PERSON_RAD**2, 0, 
                      0, PERSON_RAD**2]
        
        super().__init__(x, y, flat_covar, t)


@dataclass
class SignatureQueue:
    def __init__(self, name, stale_ms):
        self.name = name
        self.stale_ms = stale_ms # 100?

        self.queue = []
    
    def add(self, signature):
        self.queue.append(deepcopy(signature))

    def clean_out(self, current_t_ms):
        before = len(self.queue)
        self.queue = [s for s in self.queue if s.age(current_t_ms) <= self.stale_ms]
        c_rem = before - len(self.queue)

        # if c_rem > 0:
        #     print(f"Removed {c_rem} from {self.name}...")

    @property
    def values(self):
        return self.queue

    @property
    def size(self):
        return len(self.queue)


