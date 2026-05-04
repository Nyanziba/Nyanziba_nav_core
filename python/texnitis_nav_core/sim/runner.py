"""Episode runner: tick the controller, integrate the robot, log history."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

from .. import _core
from .disturbance import Disturbance, DisturbanceConfig
from .kinematics import step_holonomic, step_unicycle
from .metrics import History
from .world import World


# A controller protocol: anything with `compute_command(pose) -> (status, twist)`,
# `set_plan(path)`, `is_goal_reached(d_tol, a_tol)`, `reset()`. The pybind11
# bindings already match that shape.
Controller = object


@dataclass
class EpisodeConfig:
    """Knobs for a single ``run_episode`` call."""

    dt: float = 0.05
    max_steps: int = 600
    use_diff_drive: bool = False
    xy_tolerance: float = 0.05
    yaw_tolerance: float = 0.10


def run_episode(
    *,
    world: World,
    controller: Controller,
    plan: list[_core.Pose2D],
    config: EpisodeConfig,
    disturbance: Optional[Disturbance] = None,
    on_step: Optional[Callable[[int, History], None]] = None,
) -> History:
    """Drive the controller against the world until goal_reached or timeout."""
    history = History(last_path_length=len(plan))
    controller.reset()

    path = _core.Path2D()
    path.poses = list(plan)
    controller.set_plan(path)

    pose = world.robot.pose
    body_velocity = world.robot.body_velocity

    if disturbance is None:
        disturbance = Disturbance(DisturbanceConfig())

    integrate = step_unicycle if config.use_diff_drive else step_holonomic

    for step in range(config.max_steps):
        observed = disturbance.perturb_pose(pose)
        status, twist = controller.compute_command(_core.Pose2D(*observed))
        cmd = (twist.vx, twist.vy, twist.wz)

        # NaN / Inf accounting BEFORE quantization / latency so the bug
        # is attributed to the controller rather than the disturbance.
        if any(not math.isfinite(c) for c in cmd):
            history.nan_inf_steps.append(step)
            cmd = (0.0, 0.0, 0.0)

        delivered = disturbance.push_cmd(cmd)
        history.twists.append(delivered)

        pose = integrate(pose, delivered, config.dt)
        history.poses.append(pose)

        if world.occupancy.is_blocked(pose[0], pose[1]):
            history.collision_steps.append(step)

        reached = controller.is_goal_reached(config.xy_tolerance, config.yaw_tolerance) or \
                  bool(status == _core.ControllerResult.GoalReached)
        history.goal_reached_states.append(bool(reached))

        if reached and history.reach_step is None:
            history.reach_step = step

        if on_step is not None:
            on_step(step, history)

        if reached:
            break

    world.robot.pose = pose
    return history
