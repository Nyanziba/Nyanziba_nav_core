"""matplotlib visualization for simulator histories.

Headless-safe: callers should pre-set ``MPLBACKEND=Agg`` or pass
``backend='Agg'`` if they only want PNG / GIF output.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

import numpy as np

from .metrics import History
from .world import World


def render_episode(
    *,
    world: World,
    history: History,
    plan_xy: Optional[list[tuple[float, float]]] = None,
    save_png: Optional[Path] = None,
    show: bool = False,
    backend: Optional[str] = None,
) -> None:
    """Plot the occupancy grid, the planned path, and the executed trajectory."""
    if backend is not None:
        os.environ["MPLBACKEND"] = backend

    import matplotlib.pyplot as plt   # imported lazily so headless envs work

    fig, ax = plt.subplots(figsize=(6, 6))
    grid = world.occupancy.data
    extent = (
        world.occupancy.origin[0],
        world.occupancy.origin[0] + grid.shape[1] * world.occupancy.resolution,
        world.occupancy.origin[1],
        world.occupancy.origin[1] + grid.shape[0] * world.occupancy.resolution,
    )
    ax.imshow(grid, cmap="gray_r", origin="lower", extent=extent, alpha=0.4)

    if plan_xy:
        xs = [p[0] for p in plan_xy]
        ys = [p[1] for p in plan_xy]
        ax.plot(xs, ys, "b--", linewidth=1.5, label="planned")

    if history.poses:
        xs = [p[0] for p in history.poses]
        ys = [p[1] for p in history.poses]
        ax.plot(xs, ys, "g-", linewidth=1.0, label="executed")
        ax.plot(xs[0], ys[0], "go", label="start")
        ax.plot(xs[-1], ys[-1], "r*", label="end")

    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    if save_png is not None:
        save_png = Path(save_png)
        save_png.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_png, dpi=120, bbox_inches="tight")

    if show:
        plt.show()
    else:
        plt.close(fig)
