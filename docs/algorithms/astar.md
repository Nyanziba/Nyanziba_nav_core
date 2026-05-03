# A\* グリッドプランナー

> このファイルは骨格段階。M2 で実装が入る際に理論・実装・パラメータ感度を埋める。

## 解こうとする問題

- 入力: 占有グリッド `GridMapView`、開始 `Pose2D`、ゴール `Pose2D`
- 出力: 衝突しない `Path2D`、または `PlanResult::NoPathFound` 等のエラー
- 制約: 占有閾値 `occupied_threshold` を超えるセルは通行不可、`unknown_is_obstacle` で未知セル扱いを切替

## アルゴリズム要点

- 8 近傍 A\*、ヒューリスティクスは Octile 距離
- `inflation_radius` でグリッドを膨張させた一時マップに対して探索
- 同距離タイブレークは "後から探索したノードを優先" で再現性を確保

## ROS との関係

このアルゴリズムは ROS の型を一切扱わない。`nav_msgs::msg::OccupancyGrid` から
`GridMapView` への変換は呼び出し側（`texnitis_mbf_plugins` の `AStarPlanner` アダプタ）が担う。

## 計算量

- 最悪: O(W × H × log(W × H))
- 実用: 膨張後の自由空間の面積で抑えられることが多い

## パラメータ

実装後に `AStarParams` の各フィールドの意味と推奨値を記載する。

## サンプルコード

実装後に `examples/cpp/astar_demo.cpp` と `examples/python/astar_numpy_demo.py` を載せる。

## 参考文献

- Hart, P. E.; Nilsson, N. J.; Raphael, B. (1968). *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*.
- Daniel, K.; Nash, A.; Koenig, S.; Felner, A. (2010). *Theta\*: Any-Angle Path Planning on Grids*.
