# Lookahead 追従コントローラ

`texnitis::nav_core::LookaheadController` は、最近傍 + lookahead 距離で
目標 pose を選び、`(vx, vy, wz)` を比例制御で出すシンプルな追従器。
ホロノミック / 差動駆動を切替可能。

## 入出力

- 入力: `setPlan(Path2D)` で経路を渡し、`computeCommand(Pose2D, Twist2D&)`
  を周期的に呼ぶ
- 出力: `Twist2D`（差動駆動なら `vy=0`）と `ControllerResult`
- ゴール判定: 内部 `GoalChecker`（stateful XY フラグ）

## 計算フロー

1. 経路の最近傍 index を線形探索
2. そこから `lookahead_dist` だけ進めた pose を target にする
3. ホロノミック: target との body-frame 偏差から `(vx, vy)` を P 制御、
   `(wz)` は world-frame 接線方向との yaw 差から
4. 差動駆動: `wz` を yaw 差で出し、yaw 整合中は前進しない
5. XY 到達後はゴール yaw に整合する回転に切替

## 移動しながらの回頭（`rotate_while_moving`）

ホロノミック時のみ有効。従来の「到着してからその場回転」ではなく、
経路の走破率 `progress`（累積弧長ベース、0〜1）に応じて開始 yaw から
ゴール yaw へ補間しながら回る。

- 目標 yaw: `yaw(progress) = start_yaw + Δyaw * progress^k`
  （`k = rotate_while_moving_exponent`、`k > 1` でゴールに近づくほど
  急に回る。`start_yaw` は `setPlan` 後最初の `computeCommand` で
  現在姿勢から捕捉）
- 指令 wz: フィードフォワード + P 補正
  `wz = Δyaw * k * progress^(k-1) * v / L + kp_yaw * eyaw`
  （`v` は並進速度、`L` は経路全長）。FF が主体なので補間カーブへの
  追従遅れがほぼ出ない
- このモードでは `max_wz_when_moving` のクランプは適用されず、
  `max_speed_yaw` のみが上限になる

## 主なパラメータ（`LookaheadParams`）

| フィールド | 既定 | 意味 |
|---|---|---|
| `kp_xy` / `kp_yaw` | 1.0 / 1.2 | P ゲイン |
| `max_speed_xy` / `max_speed_yaw` | 0.25 / 0.8 | 速度上限 |
| `lookahead_dist` | 0.40 [m] | lookahead 距離 |
| `linear_threshold_for_wz` | 0.01 | 並進中の旋回抑制スレ |
| `max_wz_when_moving` | 0.0 | 並進中の最大 wz |
| `use_diff_drive` | false | 差動駆動切替 |
| `rotate_while_moving` | false | 移動中にゴール yaw へ補間回頭（ホロノミックのみ） |
| `rotate_while_moving_exponent` | 2.0 | 補間カーブの冪指数（大きいほど終盤に急回頭） |
| `goal_checker.{xy,yaw}_tolerance` | 0.05 / 0.10 | 到達許容 |
| `goal_checker.stateful` | true | XY 到達後に剥がさない |

## サンプル（Python）

```python
import texnitis_nav_core as nc

params = nc.LookaheadParams()
params.lookahead_dist = 0.30
controller = nc.LookaheadController(params)

path = nc.Path2D()
path.poses = [nc.Pose2D(0.1*i, 0.0, 0.0) for i in range(10)]
controller.set_plan(path)

status, twist = controller.compute_command(nc.Pose2D(0.0, 0.0, 0.0))
print(status, twist.vx, twist.wz)
```

## サンプル（C++）

```cpp
#include <texnitis_nav_core/controllers/lookahead_controller.hpp>

texnitis::nav_core::LookaheadParams params;
params.use_diff_drive = false;
texnitis::nav_core::LookaheadController controller (params);

texnitis::nav_core::Path2D path;
// path.poses を埋める
controller.setPlan (path);

texnitis::nav_core::Twist2D cmd;
auto status = controller.computeCommand ({0.0, 0.0, 0.0}, cmd);
```

## 参考文献

- Coulter, R. C. (1992). *Implementation of the Pure Pursuit Path Tracking
  Algorithm*. CMU-RI-TR-92-01.
