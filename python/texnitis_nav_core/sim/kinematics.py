"""Robot kinematics for the simulator.

We provide two pure-Python integrators (unicycle and mecanum/holonomic)
so the simulator stays self-contained, but for full numerical parity
with the C++ MPPI port the mecanum integrator delegates to the bound
``texnitis_nav_core.mppi.MecanumKinematics`` when it is available.
"""

from __future__ import annotations

import math
from typing import Tuple

from .. import _core, mppi as _mppi_module


def step_unicycle(pose: Tuple[float, float, float],
                  twist: Tuple[float, float, float],
                  dt: float) -> Tuple[float, float, float]:
    """Differential-drive forward integration. `vy` is ignored."""
    x, y, yaw = pose
    vx, _, wz = twist
    nx = x + vx * math.cos(yaw) * dt
    ny = y + vx * math.sin(yaw) * dt
    nyaw = (yaw + wz * dt + math.pi) % (2.0 * math.pi) - math.pi
    return nx, ny, nyaw


def step_holonomic(pose: Tuple[float, float, float],
                   twist: Tuple[float, float, float],
                   dt: float) -> Tuple[float, float, float]:
    """Mecanum / holonomic forward integration in the body frame."""
    x, y, yaw = pose
    vx, vy, wz = twist
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    nx = x + (vx * cos_y - vy * sin_y) * dt
    ny = y + (vx * sin_y + vy * cos_y) * dt
    nyaw = (yaw + wz * dt + math.pi) % (2.0 * math.pi) - math.pi
    return nx, ny, nyaw


def step_mecanum_via_core(pose: Tuple[float, float, float],
                          body_velocity: Tuple[float, float, float],
                          accel: Tuple[float, float, float],
                          dt: float,
                          v_max: float = 0.5,
                          omega_max: float = 1.5) -> Tuple[Tuple[float, float, float],
                                                            Tuple[float, float, float]]:
    """Delegate to the C++ MecanumKinematics for numerical parity.

    Returns the next (pose, body_velocity).
    """
    if _mppi_module is None:
        raise RuntimeError("MPPI bindings not built; rebuild with NAV_CORE_WITH_MPPI=ON")
    kin = _mppi_module.MecanumKinematics(v_max, omega_max, dt)
    state = [pose[0], pose[1], pose[2], body_velocity[0], body_velocity[1], body_velocity[2]]
    next_state = kin.update_state(state, list(accel))
    next_pose = (float(next_state[0]), float(next_state[1]), float(next_state[2]))
    next_vel = (float(next_state[3]), float(next_state[4]), float(next_state[5]))
    return next_pose, next_vel


__all__ = [
    "step_unicycle",
    "step_holonomic",
    "step_mecanum_via_core",
]
