# シミュレータガイド

`texnitis_nav_core.sim` は **ROS なしで** ロボットを走らせる
2D シミュレータ。pytest からシナリオ駆動でメトリクスを assert する形で
使うことを想定している。

## コンポーネント

- `OccupancyMap` — numpy 配列ベースの占有グリッド。`from_strings` で
  ASCII から作れる
- `Robot2D` — pose と body_velocity を持つ可変状態
- `World` — 上 2 つの束
- `Disturbance` / `DisturbanceConfig` — 実機相当の劣化（pose ノイズ /
  cmd_vel 遅延 / 量子化）
- `run_episode(world, controller, plan, config, disturbance)` — エピソード駆動
- `History` + `goal_reach_flip_count`/`cmd_vel_jerk_max`/`cmd_vel_rms_at_rest`/
  `path_curvature_jump_max`/`collision_count`/`nan_inf_count` — メトリクス
- `render_episode` — matplotlib 可視化（ヘッドレス対応）

## 最小エピソード

```python
import texnitis_nav_core as nc
from texnitis_nav_core.sim import (
    OccupancyMap, Robot2D, World,
    EpisodeConfig, run_episode,
)

occ = OccupancyMap.from_strings([".........."] * 10, resolution=0.1)
world = World(occupancy=occ, robot=Robot2D(pose=(0.10, 0.10, 0.0)))

planner = nc.AStarPlanner(nc.AStarParams())
_, path = planner.plan_path(occ.to_grid_map_view(),
                             nc.Pose2D(0.10, 0.10, 0.0),
                             nc.Pose2D(0.85, 0.85, 0.0))

controller = nc.LookaheadController(nc.LookaheadParams())
history = run_episode(world=world, controller=controller,
                      plan=path.poses,
                      config=EpisodeConfig(dt=0.05, max_steps=400))
print("reached at step", history.reach_step)
```

## Disturbance: 実機相当の劣化を加える

```python
from texnitis_nav_core.sim import Disturbance, DisturbanceConfig

cfg = DisturbanceConfig(
    pose_noise_xy=0.02,        # localization ノイズ stddev [m]
    pose_noise_yaw=0.05,       # yaw ノイズ stddev [rad]
    cmd_vel_latency_steps=2,   # 制御遅延 (= 2 * dt = 0.10s)
    cmd_vel_quant_linear=0.02, # CAN ステップ [m/s]
    cmd_vel_quant_angular=0.05,# CAN ステップ [rad/s]
    seed=1234,
)
history = run_episode(..., disturbance=Disturbance(cfg))
```

## メトリクス

`run_episode` は `History` を返す。代表的なメトリクスは:

- `goal_reach_flip_count(history)` — `xy_goal_reached_` が False→True
  に遷移した回数。stateful フラグが正しければ 1。
- `cmd_vel_jerk_max(history, dt)` — cmd_vel の有限差分 jerk の Linf。
  S-13 / S-14 の閾値判定で使う。
- `cmd_vel_rms_at_rest(history, last_n)` — 末尾 N サンプルの RMS。
  S-16 で localization ノイズ下の chatter を測る。
- `path_curvature_jump_max(path)` — path 上の隣接セグメント曲率変化の Linf。
- `collision_count(history)`, `nan_inf_count(history)`。

## CI（headless）

```bash
MPLBACKEND=Agg pytest -m sim
```

シナリオは pytest マーカー `sim` を持つ。
詳細は `tests_python/test_sim_scenarios.py` を参照。
