"""Minimal examples; replace step() with your own planning/control algorithm."""
from super_sim_dynamics.autonomy import DriveCommand


class StopController:
    """Default controller: hold the brakes without moving the vehicle."""
    def step(self, observation, dt):
        return DriveCommand.stop()


class CruiseController:
    """Straight-line 2 m/s demo. No obstacle avoidance; use a clear simulated road."""
    def step(self, observation, dt):
        error = 2.0 - observation.speed
        return DriveCommand(throttle=min(0.25, max(0.0, 0.3 * error)),
                            brake=min(1.0, max(0.0, -0.3 * error)))
