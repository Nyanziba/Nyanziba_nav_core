"""2D scenario-driven simulator for texnitis_nav_core.

Use ``run_episode`` from :mod:`texnitis_nav_core.sim.runner` to drive a
controller against a static occupancy map. The simulator depends on
the native ``_core`` extension being importable.
"""

from .disturbance import Disturbance, DisturbanceConfig
from .metrics import (
    History,
    cmd_vel_jerk_max,
    cmd_vel_rms_at_rest,
    collision_count,
    goal_reach_flip_count,
    nan_inf_count,
    path_curvature_jump_max,
)
from .runner import EpisodeConfig, run_episode
from .visualize import render_episode
from .world import OccupancyMap, Robot2D, World

__all__ = [
    "Disturbance",
    "DisturbanceConfig",
    "EpisodeConfig",
    "History",
    "OccupancyMap",
    "Robot2D",
    "World",
    "cmd_vel_jerk_max",
    "cmd_vel_rms_at_rest",
    "collision_count",
    "goal_reach_flip_count",
    "nan_inf_count",
    "path_curvature_jump_max",
    "render_episode",
    "run_episode",
]
