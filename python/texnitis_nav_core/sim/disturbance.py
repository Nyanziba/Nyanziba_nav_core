"""Real-robot disturbance models: pose noise, control delay, quantization."""

from __future__ import annotations

import collections
import dataclasses
from typing import Tuple

import numpy as np


@dataclasses.dataclass
class DisturbanceConfig:
    """Per-step disturbance applied during a simulator run."""

    pose_noise_xy: float = 0.0       # [m]: stddev added to reported pose (x, y)
    pose_noise_yaw: float = 0.0      # [rad]: stddev added to reported yaw
    cmd_vel_latency_steps: int = 0   # k: cmd_vel delivered after k steps
    cmd_vel_quant_linear: float = 0.0  # [m/s]: snap to multiples of this
    cmd_vel_quant_angular: float = 0.0  # [rad/s]: snap to multiples of this
    seed: int = 1234


class Disturbance:
    """Stateful disturbance applicator.

    Wraps an RNG so the same `seed` produces identical noise sequences,
    plus a deque-based delay buffer for cmd_vel.
    """

    def __init__(self, config: DisturbanceConfig) -> None:
        """Construct from a disturbance config."""
        self.config = config
        self._rng = np.random.default_rng(config.seed)
        depth = max(0, config.cmd_vel_latency_steps)
        self._cmd_buffer: collections.deque[Tuple[float, float, float]] = collections.deque(
            [(0.0, 0.0, 0.0)] * depth, maxlen=depth + 1
        )

    def perturb_pose(self,
                     pose: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Return `pose` with Gaussian noise applied to (x, y, yaw)."""
        cfg = self.config
        if cfg.pose_noise_xy <= 0.0 and cfg.pose_noise_yaw <= 0.0:
            return pose
        nx = pose[0] + self._rng.normal(0.0, cfg.pose_noise_xy)
        ny = pose[1] + self._rng.normal(0.0, cfg.pose_noise_xy)
        nyaw = pose[2] + self._rng.normal(0.0, cfg.pose_noise_yaw)
        return float(nx), float(ny), float(nyaw)

    def push_cmd(self, cmd: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Push `cmd` into the latency buffer; return the cmd delivered now.

        With latency_steps=0 this is a pass-through.
        """
        cfg = self.config
        cmd = self._quantize(cmd)
        if cfg.cmd_vel_latency_steps <= 0:
            return cmd
        # Append the new cmd; pop the oldest (= the one that takes effect now).
        self._cmd_buffer.append(cmd)
        if len(self._cmd_buffer) <= cfg.cmd_vel_latency_steps:
            return (0.0, 0.0, 0.0)
        out = self._cmd_buffer.popleft()
        return out

    def _quantize(self, cmd: Tuple[float, float, float]) -> Tuple[float, float, float]:
        cfg = self.config
        vx, vy, wz = cmd
        if cfg.cmd_vel_quant_linear > 0.0:
            vx = round(vx / cfg.cmd_vel_quant_linear) * cfg.cmd_vel_quant_linear
            vy = round(vy / cfg.cmd_vel_quant_linear) * cfg.cmd_vel_quant_linear
        if cfg.cmd_vel_quant_angular > 0.0:
            wz = round(wz / cfg.cmd_vel_quant_angular) * cfg.cmd_vel_quant_angular
        return float(vx), float(vy), float(wz)
