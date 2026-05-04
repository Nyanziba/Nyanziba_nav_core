"""Per-episode metrics computed from the simulator history."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class History:
    """Time-series log of one simulator run."""

    poses: List[Tuple[float, float, float]] = field(default_factory=list)
    twists: List[Tuple[float, float, float]] = field(default_factory=list)
    goal_reached_states: List[bool] = field(default_factory=list)
    collision_steps: List[int] = field(default_factory=list)
    nan_inf_steps: List[int] = field(default_factory=list)
    reach_step: Optional[int] = None
    last_path_length: int = 0


def goal_reach_flip_count(history: History) -> int:
    """Count False -> True transitions (a goal becoming reached)."""
    flips = 0
    prev = False
    for cur in history.goal_reached_states:
        if cur and not prev:
            flips += 1
        prev = cur
    return flips


def cmd_vel_jerk_max(history: History, dt: float) -> float:
    """Max |Δcmd_vel| / dt across the run (Linf jerk approximation)."""
    if len(history.twists) < 2 or dt <= 0.0:
        return 0.0
    peak = 0.0
    for i in range(1, len(history.twists)):
        prev = history.twists[i - 1]
        cur = history.twists[i]
        diff = max(abs(cur[0] - prev[0]), abs(cur[1] - prev[1]), abs(cur[2] - prev[2]))
        peak = max(peak, diff / dt)
    return peak


def cmd_vel_rms_at_rest(history: History, last_n: int = 20) -> float:
    """RMS of the final `last_n` cmd_vel samples; meant for stationary tail."""
    if not history.twists:
        return 0.0
    tail = history.twists[-last_n:]
    sq = 0.0
    for vx, vy, wz in tail:
        sq += vx * vx + vy * vy + wz * wz
    return math.sqrt(sq / len(tail))


def path_curvature_jump_max(path: List[Tuple[float, float, float]]) -> float:
    """Max |Δκ| between consecutive segments in `path`.

    Curvature κ on a 2-segment polyline is approximated by 2 * sin(Δθ)
    over the shorter side length. Returns 0 if the path is too short.
    """
    if len(path) < 3:
        return 0.0
    peak = 0.0
    prev_kappa = 0.0
    for i in range(1, len(path) - 1):
        ax, ay, _ = path[i - 1]
        bx, by, _ = path[i]
        cx, cy, _ = path[i + 1]
        v1 = (bx - ax, by - ay)
        v2 = (cx - bx, cy - by)
        len1 = math.hypot(*v1)
        len2 = math.hypot(*v2)
        if len1 < 1e-9 or len2 < 1e-9:
            continue
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        sin_theta = max(-1.0, min(1.0, cross / (len1 * len2)))
        kappa = 2.0 * sin_theta / max(len1, len2)
        peak = max(peak, abs(kappa - prev_kappa))
        prev_kappa = kappa
    return peak


def collision_count(history: History) -> int:
    """Number of distinct steps where the robot occupied an occupied cell."""
    return len(history.collision_steps)


def nan_inf_count(history: History) -> int:
    """Number of cmd_vel components that were NaN or Inf."""
    return len(history.nan_inf_steps)


__all__ = [
    "History",
    "goal_reach_flip_count",
    "cmd_vel_jerk_max",
    "cmd_vel_rms_at_rest",
    "path_curvature_jump_max",
    "collision_count",
    "nan_inf_count",
]
