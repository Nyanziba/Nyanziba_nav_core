# シミュレータガイド

`texnitis_nav_core.sim` は **ROS なしで** ロボットを走らせる 2D シミュレータ。
pytest からシナリオ駆動でメトリクスを assert する用途と、
`demo/python/` から手動で動かして可視化する用途の両方を想定しています。

## いきなり動かしたい場合

```bash
pip install '.[sim]'
cd demo/python
python simulator_demo.py --gif        # episode.gif が出る
python simulator_demo.py --noise 0.05 --latency 2 --gif
```

詳細は [demo/python/README.md](../../demo/python/README.md) を参照。

---

## シミュレーション方式

### 状態と運動学

ロボットの状態は **2D pose** `(x, y, yaw)` と **body 系速度** `(vx, vy, wz)`。
コントローラが返す `Twist2D` を 1 ステップ `dt` で離散積分してロボットを進めます。

| モード | 整合する車種 | 1 ステップ式 |
|---|---|---|
| **unicycle** (`step_unicycle`) | 差動駆動 | `x' = x + vx·cos(yaw)·dt`, `y' = y + vx·sin(yaw)·dt`, `yaw' = yaw + wz·dt` (vy は無視) |
| **holonomic** (`step_holonomic`) | メカナム / オムニ | body 速度 `(vx, vy)` を yaw で回転して world 系で積分 |
| **mecanum (core)** (`step_mecanum_via_core`) | メカナム + 動力学 | C++ コア `MecanumKinematics` に委譲。速度上限 / yaw 正規化込み |

3 番目だけは pybind11 経由で **MPPI 内部 rollout と同じ離散化式** を使います。
これにより「シミュレータの整合と MPPI の予測」が数値的にズレません。

### シミュレーションループ

`texnitis_nav_core.sim.runner.run_episode` は次を `max_steps` 回繰り返します:

```text
for step in 0..max_steps:
    observed_pose = disturbance.perturb_pose(true_pose)   # localization ノイズ
    status, twist = controller.compute_command(observed_pose)
    if any(NaN/Inf in twist): record + zero out
    delivered = disturbance.push_cmd(twist)               # 遅延 + 量子化
    true_pose = integrate(true_pose, delivered, dt)       # ★真値だけが進む
    if occupancy.is_blocked(true_pose): record collision
    reached = controller.is_goal_reached(...)
    history.append(...)
    if reached: break
```

**観測 pose と真 pose を分離** している点が重要です。
controller は観測 pose（ノイズ入り）しか見ないので、`pose_noise_xy=0.05` を加えると
実機の AMCL / EKF の揺らぎを再現できます。真 pose は衝突判定と描画にだけ使います。

### 衝突判定

`OccupancyMap.is_blocked(x, y)` は、世界座標を整数セル index に丸めて、

- セルが範囲外 → blocked
- セル値 `< 0` (unknown) → blocked
- セル値 `>= lethal_threshold` (既定 65) → blocked
- それ以外 → free

の単純実装です。半径を考慮しないので、ロボットの footprint を正確に扱いたい
場合は別途 inflation を A\* 側で済ませてください（`AStarParams.inflation_radius`）。

### 終了条件

- `controller.is_goal_reached(...)` が True を返す
  （内部の `GoalChecker` の stateful フラグ含む）
- `status == ControllerResult.GoalReached` が直接返る
- `step_count >= max_steps`

### Disturbance（実機相当の劣化）

`DisturbanceConfig` のフィールドで以下を再現できます。
すべて 0 にすると無劣化です。

| フィールド | 単位 | 効果 |
|---|---|---|
| `pose_noise_xy` | m | controller に渡す pose の x/y に Gaussian(0, σ) を毎ステップ加算 |
| `pose_noise_yaw` | rad | yaw に Gaussian(0, σ) を毎ステップ加算 |
| `cmd_vel_latency_steps` | step | `deque(maxlen=k+1)` の遅延バッファで cmd を遅延 |
| `cmd_vel_quant_linear` | m/s | linear 成分を `round / ε * ε` で量子化（CAN ステップ） |
| `cmd_vel_quant_angular` | rad/s | angular 成分を量子化 |
| `seed` | — | numpy.random.Generator の seed |

---

## API リファレンス

### コンポーネント

- `OccupancyMap(data, resolution, origin, lethal_threshold)`
  — numpy 配列ベースの占有グリッド。`from_strings(rows)` で ASCII から作れる
- `Robot2D(pose, body_velocity, radius)` — 可変状態
- `World(occupancy, robot)` — 上 2 つの束
- `Disturbance(DisturbanceConfig(...))` — 実機相当の劣化
- `EpisodeConfig(dt, max_steps, use_diff_drive, xy_tolerance, yaw_tolerance)`
- `run_episode(world, controller, plan, config, disturbance)` → `History`
- `History` — `poses`, `twists`, `goal_reached_states`, `collision_steps`,
  `nan_inf_steps`, `reach_step`
- `render_episode(world, history, plan_xy, save_png, show, backend)`

### メトリクス

| 関数 | 返り値 | 役割 |
|---|---|---|
| `goal_reach_flip_count(history)` | int | False→True 遷移数 (1 が正常) |
| `cmd_vel_jerk_max(history, dt)` | float | cmd_vel の有限差分 jerk の Linf |
| `cmd_vel_rms_at_rest(history, last_n)` | float | 末尾 N サンプルの RMS（停止時の chatter） |
| `path_curvature_jump_max(path)` | float | 隣接セグメント曲率変化の Linf |
| `collision_count(history)` | int | 占有セルを踏んだ step 数 |
| `nan_inf_count(history)` | int | NaN/Inf cmd の出現 step 数 |

---

## サンプル

### 最小エピソード

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

### Disturbance を加えてロバスト性を検証

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
history = run_episode(world=world, controller=controller,
                      plan=path.poses,
                      config=EpisodeConfig(dt=0.05, max_steps=600),
                      disturbance=Disturbance(cfg))
```

### 描画 (PNG / GIF) — yaw 矢印込み

`render_episode` は軌跡に加えて pose の yaw を矢印で表示します。
`arrow_stride` で矢印の間引きを、`draw_robot=True` で終端ロボットを
円 + heading line として描画できます。

```python
from texnitis_nav_core.sim.visualize import (
    render_episode,
    render_episode_animation,
)

# Still PNG: 軌跡 + yaw 矢印 + 終端ロボット.
render_episode(world=world, history=history,
               plan_xy=[(p.x, p.y) for p in path.poses],
               save_png="episode.png",
               arrow_stride=8,    # 8 step ごとに矢印を 1 本
               arrow_length=0.20,
               draw_robot=True)

# GIF アニメ: ロボット形状が yaw に合わせて回転する.
render_episode_animation(world=world, history=history,
                         plan_xy=[(p.x, p.y) for p in path.poses],
                         save_path="episode.gif", fps=20)
```

`demo/python/simulator_demo.py` がこの両方を呼び分けます:

```bash
python simulator_demo.py            # episode.png  (yaw 矢印 + 終端ロボット)
python simulator_demo.py --gif      # episode.gif (回転する円 + heading line)
python simulator_demo.py --arrow-stride 4  # 矢印を増やす
```

---

## CI（headless）

シナリオテストは pytest マーカー `sim` を持ちます:

```bash
MPLBACKEND=Agg pytest -m sim
```

実装は `tests_python/test_sim_scenarios.py`。GitHub Actions の `python` matrix で
ubuntu-24.04 / macos-latest × py3.11 / 3.12 の組合せが毎 PR で走ります。

---

## 関連ドキュメント

- [demo/README.md](../../demo/README.md) — デモ全体の説明とシミュレーション方式
- [demo/python/README.md](../../demo/python/README.md) — Python デモのコマンド集
- [docs/usage/parameters.md](parameters.md) — `LookaheadParams` 等の全リファレンス
- [docs/usage/move_base_flex_integration.md](move_base_flex_integration.md) — ROS 2 連携
