"""matplotlib visualization for simulator histories.

Headless-safe: callers should pre-set ``MPLBACKEND=Agg`` or pass
``backend='Agg'`` if they only want PNG / GIF output.

Two variants are exposed:

- :func:`render_episode` — single still PNG of the whole run, with
  optional per-step yaw arrows (``arrow_stride>0``) and an oriented
  marker for the final pose (``draw_robot=True``).
- :func:`render_episode_animation` — animated GIF / MP4 / live window
  where the robot is a moving wedge that rotates with yaw.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Iterable, Optional, Tuple

import numpy as np

from .metrics import History
from .world import World


def _draw_robot_marker(ax, x: float, y: float, yaw: float, *,
                       radius: float = 0.10, color: str = "tab:red",
                       label: Optional[str] = None) -> None:
    """Draw a small circle + heading line at (x, y, yaw)."""
    import matplotlib.patches as mpatches
    body = mpatches.Circle((x, y), radius=radius,
                           edgecolor=color, facecolor="none",
                           linewidth=2.0, label=label)
    ax.add_patch(body)
    head_x = x + radius * 1.6 * math.cos(yaw)
    head_y = y + radius * 1.6 * math.sin(yaw)
    ax.plot([x, head_x], [y, head_y], color=color, linewidth=2.0)


def _draw_yaw_arrows(ax, poses: Iterable[Tuple[float, float, float]],
                     stride: int, length: float, color: str) -> None:
    """Plot small ``ax.quiver`` arrows along ``poses`` every ``stride`` steps."""
    pts = list(poses)
    if not pts or stride <= 0:
        return
    sampled = pts[::stride]
    if not sampled:
        return
    xs = np.array([p[0] for p in sampled])
    ys = np.array([p[1] for p in sampled])
    us = np.array([math.cos(p[2]) for p in sampled])
    vs = np.array([math.sin(p[2]) for p in sampled])
    ax.quiver(xs, ys, us, vs,
              angles="xy", scale_units="xy", scale=1.0 / max(length, 1e-6),
              width=0.004, color=color, alpha=0.85, zorder=5)


def render_episode(
    *,
    world: World,
    history: History,
    plan_xy: Optional[list[tuple[float, float]]] = None,
    save_png: Optional[Path] = None,
    show: bool = False,
    backend: Optional[str] = None,
    arrow_stride: int = 8,
    arrow_length: float = 0.20,
    draw_robot: bool = True,
    robot_radius: float = 0.10,
) -> None:
    """Plot the occupancy grid, the planned path, and the executed trajectory.

    Args:
        world: Static world (occupancy grid backs the figure).
        history: Episode result from :func:`run_episode`.
        plan_xy: Optional (x, y) list of the planned path to overlay.
        save_png: When set, write the figure to this path.
        show: When True, open an interactive matplotlib window.
        backend: Force a matplotlib backend (e.g. "Agg" for headless).
        arrow_stride: Plot a yaw arrow every N executed poses; 0 disables.
        arrow_length: Arrow length in [m].
        draw_robot: Draw a circle + heading line at the final pose.
        robot_radius: Visual robot radius in [m] (does not affect collision).
    """
    if backend is not None:
        os.environ["MPLBACKEND"] = backend

    import matplotlib.pyplot as plt   # imported lazily so headless envs work

    fig, ax = plt.subplots(figsize=(8, 5))
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
        ax.plot(xs, ys, "g-", linewidth=1.2, label="executed")
        ax.plot(xs[0], ys[0], "go", label="start")
        if arrow_stride > 0:
            _draw_yaw_arrows(ax, history.poses, arrow_stride, arrow_length,
                             color="tab:green")
        if draw_robot:
            fx, fy, fyaw = history.poses[-1]
            _draw_robot_marker(ax, fx, fy, fyaw,
                               radius=robot_radius, color="tab:red",
                               label="final pose")
        else:
            ax.plot(xs[-1], ys[-1], "r*", label="end")

    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    if save_png is not None:
        save_png = Path(save_png)
        save_png.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(str(save_png), dpi=120, bbox_inches="tight")

    if show:
        plt.show()
    else:
        plt.close(fig)


def render_episode_animation(
    *,
    world: World,
    history: History,
    plan_xy: Optional[list[tuple[float, float]]] = None,
    save_path: Optional[Path] = None,
    show: bool = False,
    fps: int = 20,
    backend: Optional[str] = None,
    robot_radius: float = 0.10,
) -> None:
    """Animate the run with a robot wedge that rotates as the pose advances.

    Args:
        save_path: When set, write a GIF (``.gif``) or MP4 (``.mp4``).
        show: When True, also pop up a live window.
        fps: Frames per second.
        robot_radius: Visual robot radius for the wedge marker.
    """
    if backend is not None:
        os.environ["MPLBACKEND"] = backend

    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib import animation

    fig, ax = plt.subplots(figsize=(8, 5))
    grid = world.occupancy.data
    extent = (
        world.occupancy.origin[0],
        world.occupancy.origin[0] + grid.shape[1] * world.occupancy.resolution,
        world.occupancy.origin[1],
        world.occupancy.origin[1] + grid.shape[0] * world.occupancy.resolution,
    )
    ax.imshow(grid, cmap="gray_r", origin="lower", extent=extent, alpha=0.4)

    if plan_xy:
        ax.plot([p[0] for p in plan_xy], [p[1] for p in plan_xy],
                "b--", linewidth=1.5, label="planned")

    traj_line, = ax.plot([], [], "g-", linewidth=1.2, label="executed")
    body = mpatches.Circle((0.0, 0.0), radius=robot_radius,
                           edgecolor="tab:red", facecolor="none",
                           linewidth=2.0, label="robot")
    ax.add_patch(body)
    heading_line, = ax.plot([], [], "r-", linewidth=2.0)

    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    poses = history.poses
    if not poses:
        if save_path is not None:
            fig.savefig(Path(save_path).with_suffix(".png"), dpi=120, bbox_inches="tight")
        plt.close(fig)
        return

    xs = [p[0] for p in poses]
    ys = [p[1] for p in poses]
    yaws = [p[2] for p in poses]

    def _update(idx: int):
        traj_line.set_data(xs[: idx + 1], ys[: idx + 1])
        body.center = (xs[idx], ys[idx])
        head_x = xs[idx] + robot_radius * 1.6 * math.cos(yaws[idx])
        head_y = ys[idx] + robot_radius * 1.6 * math.sin(yaws[idx])
        heading_line.set_data([xs[idx], head_x], [ys[idx], head_y])
        return traj_line, body, heading_line

    ani = animation.FuncAnimation(fig, _update, frames=len(poses),
                                  interval=int(1000 / max(fps, 1)),
                                  blit=False)

    if save_path is not None:
        save_path = Path(save_path)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        suffix = save_path.suffix.lower()
        try:
            if suffix == ".mp4":
                ani.save(str(save_path), writer="ffmpeg", fps=fps)
            else:
                ani.save(str(save_path), writer="pillow", fps=fps)
        except Exception as exc:
            png = save_path.with_suffix(".png")
            print(f"animation save failed ({exc}); writing still {png}")
            fig.savefig(str(png), dpi=120, bbox_inches="tight")

    if show:
        plt.show()
    else:
        plt.close(fig)
