# GoalChecker

`texnitis::nav_core::GoalChecker` は XY/yaw の到達判定を行う薄いクラス。
**stateful フラグ** を持つのが特徴で、一度 XY 到達したら `reset()` まで
False に戻らない。

## 動機

旧ノードでの実機運用で、yaw 整合中に localization が振れて XY が
僅かに許容超過すると並進指令が再開してしまう問題があった。
`stateful=true` のとき、一度立った XY 到達フラグは reset まで保持
されるので、yaw 整合中の wobble に強い。

シナリオ S-01 / S-17 がこの不変条件を pin している。

## API

| メソッド | 説明 |
|---|---|
| `isXYReached(current, goal)` | XY 距離が許容内なら True。stateful なら剥がさない |
| `isYawReached(current, goal)` | yaw 差が許容内なら True（stateless） |
| `isReached(...)` | 上 2 つの AND |
| `isReached(current, goal, xy_override, yaw_override)` | mbf からの override 用 |
| `reset()` | stateful フラグをクリア |

## パラメータ

| フィールド | 既定 |
|---|---|
| `xy_tolerance` | 0.05 |
| `yaw_tolerance` | 0.10 |
| `stateful` | true |

