"""
MTRX4701 2026 Assignment 4
File: types.py
Author(s): Jeremy Fox

Custom datatypes for mahlanobis based sensor fusion

"""

from dataclasses import dataclass
import numpy as np
from copy import deepcopy

PERSON_RAD = 0.15   # std. deviation of person postition from LiDAR Ankle Detection

@dataclass
class Signature:
    """
    An (x,y) position with covariance values and an age (ms) functionality. 
    Is treated basically as an abstract base class.
    """
    x: float
    y: float
    t: float

    def __init__(self, x:float, y:float, flat_covariance, t_birth_ms:float):
        self.x = x
        self.y = y
        
        self.covariance = flat_covariance

        self.t = t_birth_ms

    def age(self, t_ms):
        return t_ms - self.t

    def __add__(self, other):
        """
        Make a signature that is the 'overlap' (average?) of
        two signatures
        """

        avg_x = (self.x + other.x) / 2.0
        avg_y = (self.y + other.y) / 2.0

        cov_a = np.array(self.covariance, dtype=float)
        cov_b = np.array(other.covariance, dtype=float)
        avg_cov = ((cov_a + cov_b) / 2.0).tolist()

        t_birth = max(self.t, other.t)

        return Signature(avg_x, avg_y, avg_cov, t_birth)
    
@dataclass
class LidarAnkleSignature(Signature):
    """
    People detections don't have a covariance,
    so give them one that reflects what we'd 
    expect from a person
    """
    def __init__(self, x, y, t):
        
        flat_covar = [PERSON_RAD**2, 0, 
                      0, PERSON_RAD**2]
        
        super().__init__(x, y, flat_covar, t)

@dataclass
class RadarPersonSignature(Signature):
    """
    This is more here for completeness sake...
    """
    def __init__(self, x, y, flat_covariance, t):
        super().__init__(x, y, flat_covariance, t)


@dataclass
class SignatureQueue:
    """
    This is a 'time based queue', any signatures that 
    are older than the stale time get removed. Didn't just
    use a Deque as I didn't know how this sort of time based 
    removal going.
    """

    def __init__(self, name, stale_ms):
        self.name = name
        self.stale_ms = stale_ms # 100?

        self.queue = []
    
    def add(self, signature):
        self.queue.append(deepcopy(signature))

    def clean_out(self, current_t_ms):

        # not entirely sure if this is handling the deepcopy alloc properly???
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


