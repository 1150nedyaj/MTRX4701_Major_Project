from dataclasses import dataclass
from abc import ABC
from math import sqrt
from copy import deepcopy

class Signature(ABC):
    def __init__(self, x:float, y:float, flat_covariance, t_birth_ms:float):
        self.x = x
        self.y = y
        
        self.covariance = flat_covariance

        self.t = t_birth_ms

    def age(self, t_ms):
        return t_ms - self.t

    # @abstractmethod
    # def area(self):
    #     """Must be implemented by subclasses"""
    #     pass

@dataclass
class RadarPersonSignature(Signature):
    def __init__(self, x, y, flat_covariance, t):
        super().__init__(x, y, flat_covariance, t)


@dataclass
class LidarAnkleSignature(Signature):
    def __init__(self, x, y, t):
        PERSON_RAD = 0.3

        flat_covar = [sqrt(PERSON_RAD), 0, 
                      0, sqrt(PERSON_RAD)]
        
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
    def size(self):
        return len(self.queue)


