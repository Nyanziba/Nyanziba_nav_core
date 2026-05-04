#!/usr/bin/env python3
"""Demo: A* で計画 → LookaheadController で追従 → matplotlib で描画.

実行:
    python astar_lookahead.py            # 既定: PNG を output/ に保存
    python astar_lookahead.py --show     # GUI で表示
    python astar_lookahead.py --map maps/corridor.yaml
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
from typing import Tuple

import numpy as np
import yaml

import texnitis_nav_core as nc


def load_map(path: Path) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    """maps/*.yaml をパースして (data, resolution, origin) を返す."""
    spec = yaml.safe_load(path.read_text())
    table = {".": 0, "#": 100, "?": -1}
    rows = [r for r in spec["occupancy"].strip("\n").splitlines() if r]
    h = len(rows)
    w = len(rows[0]) if rows else 0
    data = np.zeros((h, w), dtype=np.int8)
    for y, row in enumerate(rows):
        for x, ch in enumerate(row):
            data[y, x] = table.get(ch, 0)
    origin = tuple(spec.get("origin", [0.0, 0.0]))
    return data, float(spec["resolution"]), origin


def step_holonomic(pose: Tuple[float, float, float],
                   twist: Tuple[float, float, float],
                   dt: float) -> Tuple[float, float, float]:
    """Holonomic 1 ステップ積分（メカナム / オムニ向け）."""
    x, y, yaw = pose
    vx, vy, wz = twist
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    nx = x + (vx * cos_y - vy * sin_y) * dt
    ny = y + (vx * sin_y + vy * cos_y) * dt
    nyaw = (yaw + wz * dt + math.pi) % (2.0 * math.pi) - math.pi
    return nx, ny, nyaw


def main() -> int:
    """Run the demo."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=Path(__file__).parent / "maps" / "corridor.yaml")
    parser.add_argument("--start", nargs=3, type=float, default=[0.5, 0.5, 0.0],
                        metavar=("X", "Y", "YAW"))
    parser.add_argument("--goal", nargs=3, type=float, default=[4.5, 2.5, 0.0],
                        metavar=("X", "Y", "YAW"))
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--max-steps", type=int, default=600)
    parser.add_argument("--show", action="store_true", help="open an interactive matplotlib window")
    parser.add_argument("--save-png", type=Path,
                        default=Path(__file__).parent / "output" / "astar_lookahead.png")
    args = parser.parse_args()

    if not args.show:
        os.environ.setdefault("MPLBACKEND", "Agg")

    grid, resolution, origin = load_map(args.map)
    view = nc.GridMapView.from_numpy(grid, resolution=resolution,
                                     origin_x=origin[0], origin_y=origin[1])

    aparams = nc.AStarParams()
    aparams.inflation_radius = 0.20
    planner = nc.AStarPlanner(aparams)
    status, path = planner.plan_path(view,
                                     nc.Pose2D(*args.start),
                                     nc.Pose2D(*args.goal))
    if status != nc.PlanResult.Success:
        print(f"planner failed: {status}")
        return 1
    print(f"planned {len(path)} poses")

    cparams = nc.LookaheadParams()
    cparams.use_diff_drive = False
    cparams.lookahead_dist = 0.40
    cparams.max_speed_xy = 0.40
    cparams.max_speed_yaw = 1.20
    cparams.goal_checker.xy_tolerance = 0.10
    cparams.goal_checker.yaw_tolerance = 0.30
    cparams.goal_checker.stateful = True
    controller = nc.LookaheadController(cparams)
    controller.set_plan(path)

    pose = tuple(args.start)
    trajectory: list[Tuple[float, float, float]] = []
    for step in range(args.max_steps):
        status, twist = controller.compute_command(nc.Pose2D(*pose))
        trajectory.append(pose)
        if status == nc.ControllerResult.GoalReached:
            print(f"goal reached at step {step}")
            break
        if status != nc.ControllerResult.Success:
            print(f"controller failed: {status}")
            break
        pose = step_holonomic(pose, (twist.vx, twist.vy, twist.wz), args.dt)

    # 描画.
    import matplotlib.patches as mpatches
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 5))
    extent = (origin[0], origin[0] + grid.shape[1] * resolution,
              origin[1], origin[1] + grid.shape[0] * resolution)
    ax.imshow(grid, cmap="gray_r", origin="lower", extent=extent, alpha=0.4)
    if path.poses:
        ax.plot([p.x for p in path.poses], [p.y for p in path.poses],
                "b--", lw=1.5, label="A* plan")
    if trajectory:
        ax.plot([p[0] for p in trajectory], [p[1] for p in trajectory],
                "g-", lw=1.5, label="Lookahead trajectory")
        ax.plot(trajectory[0][0], trajectory[0][1], "go", label="start")
        # 軌跡上の yaw を quiver で間引きながら描画.
        stride = max(1, len(trajectory) // 25)
        sampled = trajectory[::stride]
        xs_a = np.array([p[0] for p in sampled])
        ys_a = np.array([p[1] for p in sampled])
        us = np.array([math.cos(p[2]) for p in sampled])
        vs = np.array([math.sin(p[2]) for p in sampled])
        ax.quiver(xs_a, ys_a, us, vs,
                  angles="xy", scale_units="xy", scale=1.0 / 0.20,
                  width=0.004, color="tab:green", alpha=0.85, zorder=5)
        # 終端のロボットを円 + heading line で描画.
        fx, fy, fyaw = trajectory[-1]
        body = mpatches.Circle((fx, fy), radius=0.10,
                               edgecolor="tab:red", facecolor="none",
                               linewidth=2.0, label="final pose")
        ax.add_patch(body)
        ax.plot([fx, fx + 0.16 * math.cos(fyaw)],
                [fy, fy + 0.16 * math.sin(fyaw)],
                color="tab:red", linewidth=2.0)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("A* + Lookahead demo (with yaw arrows)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    args.save_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(args.save_png), dpi=120, bbox_inches="tight")
    print(f"wrote {args.save_png}")
    if args.show:
        plt.show()
    plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
