#!/usr/bin/env python3
"""Demo: MPPI で直線経路を追従し matplotlib で描画.

実行:
    python mppi.py
    python mppi.py --show
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
from typing import Tuple

import numpy as np

import texnitis_nav_core as nc


def step_holonomic(pose: Tuple[float, float, float],
                   twist: Tuple[float, float, float],
                   dt: float) -> Tuple[float, float, float]:
    """Holonomic / mecanum 1 ステップ積分."""
    x, y, yaw = pose
    vx, vy, wz = twist
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    nx = x + (vx * cos_y - vy * sin_y) * dt
    ny = y + (vx * sin_y + vy * cos_y) * dt
    nyaw = (yaw + wz * dt + math.pi) % (2.0 * math.pi) - math.pi
    return nx, ny, nyaw


def main() -> int:
    """Run the MPPI demo."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--save-png", type=Path,
                        default=Path(__file__).parent / "output" / "mppi.png")
    parser.add_argument("--horizon", type=int, default=20)
    parser.add_argument("--num-samples", type=int, default=256)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--max-steps", type=int, default=400)
    args = parser.parse_args()

    if nc.mppi is None:
        print("MPPI bindings not built; rebuild with NAV_CORE_WITH_MPPI=ON")
        return 1
    if not args.show:
        os.environ.setdefault("MPLBACKEND", "Agg")

    # 直線パス (0.5, 0.5) -> (5.5, 1.5).
    plan = nc.Path2D()
    plan.poses = [nc.Pose2D(0.5 + 5.0 * t, 0.5 + 1.0 * t, 0.0)
                  for t in np.linspace(0.0, 1.0, 51)]

    params = nc.mppi.MppiParams()
    params.horizon = args.horizon
    params.num_samples = args.num_samples
    params.lambda_ = 0.10
    params.dt = args.dt
    params.v_max = 0.50
    params.omega_max = 1.50
    params.goal_checker.xy_tolerance = 0.10
    params.goal_checker.yaw_tolerance = 0.30
    params.goal_checker.stateful = True

    controller = nc.mppi.MecanumMppiController(params)
    controller.set_plan(plan)

    pose = (0.5, 0.5, 0.0)
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

    import matplotlib.patches as mpatches
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot([p.x for p in plan.poses], [p.y for p in plan.poses],
            "b--", lw=1.5, label="planned line")
    if trajectory:
        ax.plot([p[0] for p in trajectory], [p[1] for p in trajectory],
                "g-", lw=1.5, label="MPPI trajectory")
        ax.plot(trajectory[0][0], trajectory[0][1], "go", label="start")
        # yaw 矢印.
        stride = max(1, len(trajectory) // 25)
        sampled = trajectory[::stride]
        xs_a = np.array([p[0] for p in sampled])
        ys_a = np.array([p[1] for p in sampled])
        us = np.array([math.cos(p[2]) for p in sampled])
        vs = np.array([math.sin(p[2]) for p in sampled])
        ax.quiver(xs_a, ys_a, us, vs,
                  angles="xy", scale_units="xy", scale=1.0 / 0.20,
                  width=0.004, color="tab:green", alpha=0.85, zorder=5)
        # 終端ロボット.
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
    ax.set_title(f"MPPI demo (horizon={args.horizon}, samples={args.num_samples})")
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
