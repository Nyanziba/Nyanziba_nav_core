# Differential drive Pure Pursuit

`texnitis::nav_core::DiffDrivePurePursuitController` は、
**動的 lookahead 距離**（速度に比例）と**加速度上限**を備えた
差動駆動向け pure pursuit 追従器。

## アルゴリズム要点

1. 経路の最近傍 index を線形探索
2. lookahead 距離 = `clamp(|last_vx| * lookahead_time, [min, max])`
3. 接線方向との yaw 誤差から `wz_des` を出す
4. 並進指令 `vx_des` は `max_linear * (1 - angle_speed_p * |yaw_err|/π)`
   と最近傍までの距離の min（旋回中は減速）
5. 1 step あたり `max_acceleration * control_dt` の変化に制限（vx と wz 両方）
6. XY 到達後はゴール yaw への整合のみ行う（vx=0）

## 主なパラメータ（`DiffDrivePurePursuitParams`）

| フィールド | 既定 | 意味 |
|---|---|---|
| `max_linear_velocity` | 0.50 | 並進上限 [m/s] |
| `max_angular_velocity` | 1.00 | ヨー上限 [rad/s] |
| `max_acceleration` | 0.50 | 並進加速度上限 [m/s^2] |
| `max_angular_acceleration` | 1.50 | ヨー加速度上限 [rad/s^2] |
| `lookahead_time` | 0.80 | lookahead 比例係数 [s] |
| `min_lookahead_distance` | 0.30 | 最小 lookahead [m] |
| `max_lookahead_distance` | 1.00 | 最大 lookahead [m] |
| `angle_speed_p` | 0.80 | 旋回中の減速強度 |
| `control_dt` | 0.05 | 1 tick の dt [s]（加速度上限の計算に使う） |
| `goal_checker.*` | — | 到達判定（stateful） |

## サンプル

```cpp
texnitis::nav_core::DiffDrivePurePursuitParams params;
params.max_acceleration = 0.50;
texnitis::nav_core::DiffDrivePurePursuitController controller (params);
controller.setPlan (path);
texnitis::nav_core::Twist2D cmd;
controller.computeCommand (current_pose, cmd);
```

## 参考文献

- Coulter, R. C. (1992). *Implementation of the Pure Pursuit Path Tracking
  Algorithm*. CMU-RI-TR-92-01.
