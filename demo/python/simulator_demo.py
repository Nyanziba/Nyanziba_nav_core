#!/usr/bin/env python3
"""Demo: フル機能シミュレータ (disturbance + GIF アニメ + メトリクス).

`run_episode` で disturbance 入りエピソードを 1 本走らせ、
PNG または GIF にして保存します。`--show` で matplotlib ウィンドウ表示。

実行:
    python simulator_demo.py                   # PNG 保存
    python simulator_demo.py --gif             # GIF アニメ保存
    python simulator_demo.py --show            # 対話表示
    python simulator_demo.py --noise 0.05      # localization ノイズ stddev [m]
    python simulator_demo.py --latency 2       # cmd_vel 遅延 [steps]
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import List, Tuple

import yaml

import texnitis_nav_core as nc
from texnitis_nav_core.sim import (
    Disturbance,
    DisturbanceConfig,
    EpisodeConfig,
    OccupancyMap,
    Robot2D,
    World,
    cmd_vel_jerk_max,
    cmd_vel_rms_at_rest,
    collision_count,
    goal_reach_flip_count,
    nan_inf_count,
    run_episode,
)
from texnitis_nav_core.sim.visualize import (
    render_episode,
    render_episode_animation,
)


def load_map(path: Path) -> OccupancyMap:
    """maps/*.yaml を `OccupancyMap` にする."""
    spec = yaml.safe_load(path.read_text())
    rows = [r for r in spec["occupancy"].strip("\n").splitlines() if r]
    return OccupancyMap.from_strings(rows,
                                     resolution=float(spec["resolution"]),
                                     origin=tuple(spec.get("origin", [0.0, 0.0])))


def make_lookahead() -> nc.LookaheadController:
    """Demo 共通の Lookahead コントローラ."""
    p = nc.LookaheadParams()
    p.use_diff_drive = False
    p.lookahead_dist = 0.40
    p.max_speed_xy = 0.40
    p.max_speed_yaw = 1.20
    p.goal_checker.xy_tolerance = 0.10
    p.goal_checker.yaw_tolerance = 0.30
    p.goal_checker.stateful = True
    return nc.LookaheadController(p)


def render(world: World,
           history,
           plan_xy: List[Tuple[float, float]],
           out_path: Path,
           *,
           as_gif: bool,
           show: bool,
           arrow_stride: int) -> None:
    """yaw 矢印付きの still PNG または rotating-wedge GIF を書き出す."""
    if as_gif:
        out = out_path.with_suffix(".gif")
        render_episode_animation(
            world=world,
            history=history,
            plan_xy=plan_xy,
            save_path=out,
            show=show,
            fps=20,
        )
    else:
        out = out_path.with_suffix(".png")
        render_episode(
            world=world,
            history=history,
            plan_xy=plan_xy,
            save_png=out,
            show=show,
            arrow_stride=arrow_stride,
            arrow_length=0.20,
            draw_robot=True,
        )
    print(f"wrote {out}")


def main() -> int:
    """Run the simulator demo."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path,
                        default=Path(__file__).parent / "maps" / "corridor.yaml")
    parser.add_argument("--start", nargs=3, type=float, default=[0.5, 0.5, 0.0])
    parser.add_argument("--goal", nargs=3, type=float, default=[4.5, 2.5, 3.0])
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--max-steps", type=int, default=600)
    parser.add_argument("--noise", type=float, default=10.0,
                        help="pose ノイズ stddev [m / rad は半分]")
    parser.add_argument("--latency", type=int, default=0,
                        help="cmd_vel latency [steps]")
    parser.add_argument("--gif", action="store_true", help="save GIF instead of PNG")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--save", type=Path,
                        default=Path(__file__).parent / "output" / "episode.gif")
    parser.add_argument("--arrow-stride", type=int, default=8,
                        help="still PNG で yaw 矢印を N 点ごとに描画。0 で無効")
    args = parser.parse_args()

    if not args.show:
        os.environ.setdefault("MPLBACKEND", "Agg")

    occ = load_map(args.map)
    world = World(occupancy=occ, robot=Robot2D(pose=tuple(args.start)))

    aparams = nc.AStarParams()
    aparams.inflation_radius = 0.20
    planner = nc.AStarPlanner(aparams)
    status, plan = planner.plan_path(occ.to_grid_map_view(),
                                     nc.Pose2D(*args.start),
                                     nc.Pose2D(*args.goal))
    if status != nc.PlanResult.Success:
        print(f"planner failed: {status}")
        return 1
    print(f"planned {len(plan)} poses")

    controller = make_lookahead()
    disturbance = Disturbance(DisturbanceConfig(
        pose_noise_xy=args.noise,
        pose_noise_yaw=args.noise * 0.5,
        cmd_vel_latency_steps=args.latency,
        seed=42,
    ))

    history = run_episode(
        world=world,
        controller=controller,
        plan=plan.poses,
        config=EpisodeConfig(dt=args.dt, max_steps=args.max_steps,
                             xy_tolerance=0.10),
        disturbance=disturbance,
    )

    print(f"reach_step          = {history.reach_step}")
    print(f"goal_reach_flips    = {goal_reach_flip_count(history)}")
    print(f"cmd_vel_jerk_max    = {cmd_vel_jerk_max(history, args.dt):.3f}")
    print(f"cmd_vel_rms_at_rest = {cmd_vel_rms_at_rest(history):.4f}")
    print(f"collisions          = {collision_count(history)}")
    print(f"NaN/Inf             = {nan_inf_count(history)}")

    plan_xy = [(p.x, p.y) for p in plan.poses]
    out = args.save if args.save.suffix else (
        args.save.with_suffix(".gif") if args.gif else args.save.with_suffix(".png")
    )
    render(world, history, plan_xy, out,
           as_gif=args.gif, show=args.show, arrow_stride=args.arrow_stride)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
