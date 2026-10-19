"""ROS-independent controller contract. Units: metres, seconds, radians."""
from dataclasses import dataclass
import importlib
import math


@dataclass(frozen=True)
class Observation:
    x: float
    y: float
    yaw: float
    speed: float  # body-frame forward velocity, m/s
    yaw_rate: float  # rad/s


@dataclass(frozen=True)
class DriveCommand:
    throttle: float = 0.0  # -1..1 (negative requests reverse)
    brake: float = 0.0  # 0..1
    steering: float = 0.0  # front-wheel angle in radians; positive left
    handbrake: bool = False

    @classmethod
    def stop(cls):
        return cls(brake=1.0)

    @classmethod
    def from_array(cls, values):
        if len(values) not in (3, 4) or not all(math.isfinite(v) for v in values):
            raise ValueError("Expected 3 or 4 finite drive values")
        return cls(*values[:3], handbrake=len(values) == 4 and values[3] > 0.5)

    def limited(self, max_steering=0.65):
        if not all(math.isfinite(v) for v in (self.throttle, self.brake, self.steering)):
            raise ValueError("Controller produced a non-finite command")
        brake = min(1.0, max(0.0, self.brake))
        throttle = 0.0 if brake > 0 or self.handbrake else min(1.0, max(-1.0, self.throttle))
        return DriveCommand(throttle, brake, max(-max_steering, min(max_steering, self.steering)), bool(self.handbrake))

    def as_array(self):
        return [float(self.throttle), float(self.brake), float(self.steering), float(self.handbrake)]


def load_controller(spec):
    """Load a user-authored, importable module:Class; constructor takes no arguments."""
    module, separator, name = spec.partition(":")
    if not separator or not module or not name:
        raise ValueError("Controller must be specified as module:Class")
    controller = getattr(importlib.import_module(module), name)()
    if not callable(getattr(controller, "step", None)):
        raise TypeError("Controller must provide step(observation, dt)")
    return controller
