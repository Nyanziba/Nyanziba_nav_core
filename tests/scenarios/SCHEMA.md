# シナリオ YAML スキーマ

各シナリオは `tests/scenarios/<id>/scenario.yaml` に置き、対応する正解値を `tests/scenarios/<id>/expected.json` に固定する。同じ YAML を **C++（yaml-cpp / nlohmann_json）と Python（PyYAML / json）の両方** から読めるよう、構造はネストした dict と list のみを使う。

## トップレベル

```yaml
id: S-XX                       # tests/scenarios/<id>/ と同一
description: "..."             # 何を確認するシナリオかを 1〜2 行で
seed: 42                       # 確率的なステップ（pose ノイズ等）の再現性
tolerances:
  pose_xy: 1.0e-6              # expected.json 比較時の許容誤差
  pose_yaw: 1.0e-6
  twist: 1.0e-6
  curvature: 1.0e-3            # S-13 用
  metrics_relative: 1.0e-2     # S-14 / S-16 用、メトリクスの相対許容
trials:
  count: 1                     # 確率的シナリオは 5 程度。決定的シナリオは 1
  seed_strategy: fixed         # fixed | per_trial
plugins:
  planner:
    name: astar                # nav_core::AStarPlanner / mbf プラグイン名
    params:                    # nav_core::AStarParams に詰める POD フィールド
      occupied_threshold: 65
      inflation_radius: 0.30
      unknown_is_obstacle: true
  controller:
    name: lookahead
    params:
      kp_xy: 1.0
      kp_yaw: 1.2
      max_speed_xy: 0.25
      max_speed_yaw: 0.8
      lookahead_dist: 0.40
      goal_xy_tolerance: 0.05
      goal_yaw_tolerance: 0.10
      goal_stateful: true
map:                           # 固定占有グリッド。空白は free, '#' は occupied
  resolution: 0.10             # [m/cell]
  origin: [0.0, 0.0]
  occupancy: |
    ..........
    ..........
    ..#####...
    ..........
    ..........
height_map:                    # HeightAware* シナリオ用、任意
  resolution: 0.10
  origin: [0.0, 0.0]
  values: |                    # 0..100 の符号付き整数（int8）。'?' は unknown
    ..........
    ..........
disturbance:                   # 任意。実機相当の劣化条件
  pose_noise:
    sigma_xy: 0.0              # 0 で無効
    sigma_yaw: 0.0
  cmd_vel_latency_steps: 0     # 制御遅延（dt 単位）
  cmd_vel_quantization:        # CAN ステップ等の量子化
    linear: 0.0                # 0 で無効
    angular: 0.0
  max_acc:                     # 加速度上限
    linear: 1.0e9              # 大きな値で無効化
    angular: 1.0e9
sequence:                      # 操作列。順序通りに駆動する
  - {action: set_goal,    pose: [0.5, 0.5, 0.0]}
  - {action: drive_until, condition: goal_reached, max_steps: 300, dt: 0.05}
  - {action: assert,      key: outcome,           value: SUCCESS}
metrics:                       # 集計値の閾値（実装後に metrics.py が計算）
  path_curvature_jump_max: 5.0
  cmd_vel_jerk_max: 4.0
  cmd_vel_rms_at_rest: 0.025
  goal_reach_flip_count: 1
  collision_count: 0
  nan_inf_count: 0
```

## `sequence` のアクション一覧

| `action` | フィールド | 意味 |
|---|---|---|
| `set_goal` | `pose: [x, y, yaw]` | 新しいゴールを投入。controller の `reset()` が呼ばれる |
| `set_path` | `poses: [[x,y,yaw], ...]` | 外部 path を直接 controller に投げる（planner をバイパス） |
| `set_pose` | `pose: [x, y, yaw]` | ロボット pose を瞬間移動（テスト初期化用） |
| `cancel` | なし | 現在の goal を cancel。planner / controller の cancel フラグが立つ |
| `drive_until` | `condition`, `max_steps`, `dt` | 条件成立まで `dt` 刻みでシミュレータを進める |
| `step` | `count`, `dt` | 指定回数だけ進める（条件は問わない） |
| `wait` | `seconds` | 指定秒だけ静止状態を維持（cmd_vel=0） |
| `enable_disturbance` | `kind`, `value` | 途中から noise や latency を有効化 |
| `assert` | `key`, `value`, `tolerance?` | 期待値との一致を assert |

`condition` は `goal_reached`, `outcome != SUCCESS`, `path_received`, `cancelled`, `step_count >= N` などをサポート（M0 で実装する `runner.py` で定義）。

## `assert` の `key` 一覧

| key | 意味 |
|---|---|
| `outcome` | 最後の planner / controller 結果（`SUCCESS`, `NO_PATH_FOUND`, `CANCELLED`, `EMPTY_PATH`, ...） |
| `last_twist.{vx,vy,wz}` | 直前 step の cmd_vel |
| `pose.{x,y,yaw}` | ロボット pose |
| `goal_reach_count` | このシナリオで `goal_reached` が True に遷移した回数 |
| `collision_count` | 占有セル踏み回数 |
| `xy_goal_reached` | stateful フラグの現在値 |
| `metrics.<name>` | `metrics.py` が集計した値 |

## `expected.json` の形式

旧 `texnitis_move_base_like` を駆動して固定する正解値:

```json
{
  "id": "S-XX",
  "generated_by": "legacy_harness/run_legacy.py",
  "legacy_commit": "<old repo SHA at recording time>",
  "trials": [
    {
      "seed": 42,
      "path": [[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], "..."],
      "twist_series": [[0.10, 0.00, 0.00], "..."],
      "goal_reach_steps": [123],
      "outcomes": ["SUCCESS"],
      "metrics": {
        "goal_reach_time": 6.15,
        "path_curvature_jump_max": 0.42,
        "cmd_vel_jerk_max": 1.20,
        "cmd_vel_rms_at_rest": 0.001,
        "goal_reach_flip_count": 1,
        "collision_count": 0,
        "nan_inf_count": 0
      }
    }
  ]
}
```

許容誤差 (`tolerances.*`) を超えない範囲で、新コアの実行結果が `trials[*]` と一致することを確認する。
