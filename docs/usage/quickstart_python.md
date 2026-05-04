# Python クイックスタート

`texnitis_nav_core` を ROS なしの Python から使う最小手順。

## インストール

```bash
git clone https://github.com/Nyanziba/texnitis_nav_core.git
cd texnitis_nav_core
# Eigen が必要 (MPPI を含む wheel をビルドするため)
sudo apt install -y libeigen3-dev cmake          # Ubuntu 24.04
# brew install eigen cmake                        # macOS

python -m venv .venv && source .venv/bin/activate
pip install '.[sim,test]'
```

## 確認

```python
import texnitis_nav_core as nc
print(nc.__version__)
print("MPPI:", nc.mppi is not None)
```

## A\* + Lookahead を numpy で動かす

```python
import numpy as np
import texnitis_nav_core as nc

grid = np.zeros((10, 10), dtype=np.int8)
view = nc.GridMapView.from_numpy(grid, resolution=0.1)

planner = nc.AStarPlanner(nc.AStarParams())
status, path = planner.plan_path(view,
                                 nc.Pose2D(0.05, 0.05, 0.0),
                                 nc.Pose2D(0.85, 0.85, 0.0))
assert status == nc.PlanResult.Success

controller = nc.LookaheadController(nc.LookaheadParams())
controller.set_plan(path)
status, twist = controller.compute_command(nc.Pose2D(0.0, 0.0, 0.0))
print(status, twist.vx, twist.wz)
```

## シミュレータでロボットを走らせる

```python
import texnitis_nav_core as nc
from texnitis_nav_core.sim import (
    World, OccupancyMap, Robot2D,
    EpisodeConfig, run_episode, render_episode,
)

occ = OccupancyMap.from_strings([".........."] * 10, resolution=0.1)
world = World(occupancy=occ, robot=Robot2D(pose=(0.10, 0.10, 0.0)))

planner = nc.AStarPlanner(nc.AStarParams())
status, path = planner.plan_path(occ.to_grid_map_view(),
                                 nc.Pose2D(0.10, 0.10, 0.0),
                                 nc.Pose2D(0.85, 0.85, 0.0))

controller = nc.LookaheadController(nc.LookaheadParams())
history = run_episode(world=world, controller=controller, plan=path.poses,
                      config=EpisodeConfig(dt=0.05, max_steps=400))

print("reach_step:", history.reach_step)
render_episode(world=world, history=history, save_png="episode.png")
```

## トラブルシュート

- `import texnitis_nav_core` で `_core` が None: ビルド時に Eigen3 が見つからず
  C++ 拡張がスキップされた。`pip install '.[sim,test]'` を Eigen 入りで再実行。
- numpy ndarray の dtype が int8 でないと `GridMapView.from_numpy` で
  RuntimeError。`grid.astype(np.int8)` を使う。
- ヘッドレスで matplotlib の警告が出る: `MPLBACKEND=Agg` を環境に設定。
