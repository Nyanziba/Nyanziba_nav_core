# Kinematic Time Planner テスト根拠

Test Basis は [kinematic_time_planner_design.md](../kinematic_time_planner_design.md) の
第3、5、6、7、11、12節、および各公開ヘッダのコントラクトとする。

| Persona | 観点 | 対応テスト |
|---|---|---|
| P1 新人ユーザー | Terrainなしの通常2D Map、空の出力先をそのまま渡す | `AcceptsPlain2DMap`, `RejectsInvalidMapAndClearsPreviousTrajectory` |
| P2 ベテラン | 同一インスタンスの連続呼出し、探索上限、決定性 | `RepeatedPlanningIsDeterministic`, `IterationLimitReturnsNoPath` |
| P3 悪意ある操作者 | null map、不正モデル、NaN Terrain、範囲外・衝突start/goal | `Rejects*`, `IgnoresNanTerrainCell`, `InvalidGradientIsRejectedWhenTerrainRequired` |
| P4 整合性監査役 | occupancy、cost、yaw、時刻、終点速度、転倒余裕を直接確認 | `ElevationStepBecomesLethal`, `TrajectoryHasMonotonicTimeAndStopsAtGoal`, `RejectsLateralSlopeBeyondStaticStabilityLimit` |
| P5 移行担当者 | 従来のOccupancyGridだけで成功する | `AcceptsPlain2DMap`, 既存AStarテスト群 |
| P6 デグレ番人 | AStar/Controller既存テストを全実行 | CTest全件 |
| P7 仕様懐疑者 | 差動二輪に横移動がない、終点yaw、上り下りの非対称性、Terrainが探索へ実際に入るか | `DifferentialPrimitivesDoNotStrafe`, `PreservesGoalYaw`, `UphillTravelTakesLongerThanDownhill`, `ValidSlopeLayerParticipatesInPlanning` |

## 要確認

- slope gridとelevation由来勾配が矛盾した場合の優先規則
- Terrain欠損セルをfree/unknown/lethalのどれとして扱うか
- 独立ステアリングの舵角状態数と車輪反転コスト
- 実機で「既存AStarより速い」と判定する有意差（比較ツールは生データと中央値を保存し、推測で閾値を置かない）
- 動的転倒評価に使う最大横加速度、サスペンション、タイヤ滑りモデル

上記は仕様決定まで推測で期待値を置かない。
