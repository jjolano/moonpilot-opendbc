"""moonpilot's engine-braking coast (fork code; see the superproject's AGENTS.md).

Toyota's carcontroller sends PERMIT_BRAKING for every request under +0.2 m/s^2, so the PCM may use the
friction brakes — and light the lamps — for a request the engine alone would meet. With the fork's
`ToyotaFlags.MOONPILOT_ENGINE_COAST` set, PERMIT_BRAKING is held off while the request stays within
`ENGINE_COAST_ENTER` of the engine's own coast (`engine_coast_floor`), and restored the moment it falls
`ENGINE_COAST_EXIT` past it. Below `ENGINE_COAST_MIN_SPEED`, while stopping, or without longitudinal
control, upstream's own value stands.

The floor is the driver's measured no-pedal coast (moonpilot/longitudinal.py `MOONPILOT_COASTING_*`,
-0.30 at 5-10 m/s and -0.33 at 10-25), interpolated from the 5 m/s floor up; moonpilot's planner reads
the same curve, and moonpilot/tests/test_engine_coast.py pins the two together.
"""
import numpy as np

ENGINE_COAST_MIN_SPEED = 5.0  # m/s
ENGINE_COAST_BP = [5.0, 7.5, 15.0]  # m/s
ENGINE_COAST_V = [-0.175, -0.30, -0.33]  # m/s^2
ENGINE_COAST_ENTER = 0.05  # m/s^2 past the floor a request may sit and still hold the brakes off
ENGINE_COAST_EXIT = 0.15  # m/s^2 past the floor that gives the brakes back at once


def engine_coast_floor(v_ego: float) -> float:
  return float(np.interp(v_ego, ENGINE_COAST_BP, ENGINE_COAST_V))


class EngineCoast:
  def __init__(self):
    self.coasting = False

  def permit(self, upstream: bool, request: float, v_ego: float, active: bool) -> bool:
    """PERMIT_BRAKING for this frame, given upstream's own decision and the net accel request."""
    if not active or v_ego <= ENGINE_COAST_MIN_SPEED:
      self.coasting = False
      return upstream
    floor = engine_coast_floor(v_ego)
    if request >= floor - ENGINE_COAST_ENTER:
      self.coasting = True
    elif request < floor - ENGINE_COAST_EXIT:
      self.coasting = False
    return upstream and not self.coasting
