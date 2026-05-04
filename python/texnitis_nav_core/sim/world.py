"""2D world: occupancy map, robot pose, collision check."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np

from .. import _core


@dataclass
class OccupancyMap:
    """Static occupancy grid backing the simulator.

    `data` is an int8 ndarray of shape (height, width). Cells with values
    >= `lethal_threshold` are treated as blocked.
    """

    data: np.ndarray
    resolution: float = 0.10
    origin: Tuple[float, float] = (0.0, 0.0)
    lethal_threshold: int = 65

    @classmethod
    def from_strings(cls, rows: list[str], resolution: float = 0.10,
                     origin: Tuple[float, float] = (0.0, 0.0)) -> "OccupancyMap":
        """Build a map from ASCII rows ('#' lethal, '.' free, '?' unknown)."""
        translation = {".": 0, "#": 100, "?": -1}
        height = len(rows)
        width = len(rows[0]) if rows else 0
        data = np.zeros((height, width), dtype=np.int8)
        for y, row in enumerate(rows):
            for x, ch in enumerate(row):
                data[y, x] = translation.get(ch, 0)
        return cls(data=data, resolution=resolution, origin=origin)

    def world_to_index(self, x: float, y: float) -> Tuple[int, int]:
        """Return integer (mx, my) for world (x, y)."""
        mx = int((x - self.origin[0]) / self.resolution)
        my = int((y - self.origin[1]) / self.resolution)
        return mx, my

    def is_blocked(self, x: float, y: float) -> bool:
        """True if (x, y) lies in a lethal or out-of-bounds cell."""
        mx, my = self.world_to_index(x, y)
        if mx < 0 or my < 0 or mx >= self.data.shape[1] or my >= self.data.shape[0]:
            return True
        value = int(self.data[my, mx])
        if value < 0:
            return True   # treat unknown as blocked for the simulator
        return value >= self.lethal_threshold

    def to_grid_map_view(self) -> _core.GridMapView:
        """Build a borrowed GridMapView. Caller keeps the OccupancyMap alive."""
        return _core.GridMapView.from_numpy(
            self.data, resolution=self.resolution,
            origin_x=self.origin[0], origin_y=self.origin[1])


@dataclass
class Robot2D:
    """Mutable robot state. Pose in world frame; body_velocity in body frame."""

    pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    body_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    radius: float = 0.10

    def collides_with(self, world_map: OccupancyMap) -> bool:
        """Cell-level collision check at the robot footprint."""
        x, y, _ = self.pose
        return world_map.is_blocked(x, y)


@dataclass
class World:
    """Bundles the static map and the dynamic robot for the simulator."""

    occupancy: OccupancyMap
    robot: Robot2D = field(default_factory=Robot2D)
