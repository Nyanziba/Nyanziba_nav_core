# Mecanum 運動学

`texnitis::nav_core::mppi::MecanumKinematics` は MPPI の rollout で使う
1 ステップ離散積分器。

## 状態と制御

- 状態 `State = [x, y, theta, vx, vy, omega]`（位置は世界、速度は機体）
- 制御 `Control = [ax, ay, alpha]`（加速度・角加速度、機体座標系）

## 1 ステップ更新

```text
vx_n = vx + ax * dt
vy_n = vy + ay * dt
omega_n = omega + alpha * dt
速度ノルム制約: |(vx_n, vy_n)| <= v_max （線形にスケール）
角速度制約: |omega_n| <= omega_max
位置更新（世界系）: 機体速度を yaw で回転して x/y を積分
yaw 更新: theta_n = theta + omega_n * dt、[-π, π] に正規化
```

## ホイール速度

`getWheelSpeeds(vx, vy, omega)` は 4 輪 [FL, FR, RL, RR] を返す。
`L = wheel_base`:

| 輪 | 式 |
|---|---|
| FL | vx − vy − L · omega |
| FR | vx + vy + L · omega |
| RL | vx + vy − L · omega |
| RR | vx − vy + L · omega |

## サンプル

```cpp
texnitis::nav_core::mppi::MecanumKinematics k (0.5, 1.5, 0.05);
auto next = k.updateState (state, control);
```
