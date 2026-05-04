# パラメータ参照表

## 共通

| 構造体 | フィールド | 既定 | 単位 |
|---|---|---|---|
| `GoalCheckerParams` | `xy_tolerance` | 0.05 | m |
| | `yaw_tolerance` | 0.10 | rad |
| | `stateful` | true | — |

## Planner

### `AStarParams`

| フィールド | 既定 | 単位 |
|---|---|---|
| `occupied_threshold` | 65 | int (0-100) |
| `unknown_is_obstacle` | true | — |
| `inflation_radius` | 0.30 | m |
| `max_iterations` | 0 (auto) | int |
| `heuristic_weight` | 1.0 | — |
| `allow_diagonal` | true | — |

### `HeightAwareAStarParams`

| フィールド | 既定 |
|---|---|
| `astar` | `AStarParams` |
| `height_lethal_threshold` | 50 |
| `require_height_grid` | true |

## Controller

### `LookaheadParams`

| フィールド | 既定 | 単位 |
|---|---|---|
| `kp_xy` | 1.0 | — |
| `kp_yaw` | 1.2 | — |
| `max_speed_xy` | 0.25 | m/s |
| `max_speed_yaw` | 0.8 | rad/s |
| `lookahead_dist` | 0.40 | m |
| `linear_threshold_for_wz` | 0.01 | m/s |
| `max_wz_when_moving` | 0.0 | rad/s |
| `use_diff_drive` | false | — |
| `goal_checker` | `GoalCheckerParams` |

### `DiffDrivePurePursuitParams`

| フィールド | 既定 | 単位 |
|---|---|---|
| `max_linear_velocity` | 0.50 | m/s |
| `min_linear_velocity` | 0.05 | m/s |
| `max_angular_velocity` | 1.00 | rad/s |
| `max_acceleration` | 0.50 | m/s^2 |
| `max_angular_acceleration` | 1.50 | rad/s^2 |
| `lookahead_time` | 0.80 | s |
| `min_lookahead_distance` | 0.30 | m |
| `max_lookahead_distance` | 1.00 | m |
| `angle_speed_p` | 0.80 | — |
| `control_dt` | 0.05 | s |
| `goal_checker` | `GoalCheckerParams` |

### `MppiParams`

| フィールド | 既定 |
|---|---|
| `horizon` | 25 |
| `num_samples` | 256 |
| `lambda` | 0.10 |
| `sigma` | (0.30, 0.30, 0.40) |
| `u_max` | (1.50, 1.50, 1.50) |
| `dt` | 0.05 |
| `w_track` / `w_yaw` / `w_control` | 1.0 / 0.30 / 0.05 |
| `v_max` / `omega_max` / `wheel_base` | 0.50 / 1.50 / 0.30 |
| `seed` | 42 |
| `goal_checker` | `GoalCheckerParams` |
