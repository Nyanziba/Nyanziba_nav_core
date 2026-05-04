# A\* グリッドプランナー

## 解こうとする問題

- 入力: 占有グリッド `GridMapView`、開始 `Pose2D`、ゴール `Pose2D`
- 出力: 衝突しない `Path2D`、または `PlanResult` の失敗値
- 制約: 占有閾値 `occupied_threshold` 以上のセルは通行不可。
  `unknown_is_obstacle=true` のとき `-1` (unknown) も通行不可

## アルゴリズム要点

- **8 近傍 A\***（`allow_diagonal=true`）または **4 近傍 A\***
- ヒューリスティクス: **Octile 距離**（`max - min + sqrt(2) * min`）。
  8 近傍探索で admissible
- **inflation**: lethal セルを `inflation_radius / resolution` セル分
  膨張させた一時マップを `inflated_` に保持。再呼び出し時は同じバッファを
  再利用（メモリ確保削減）
- **タイブレーク**: `std::priority_queue<Node, vector, Greater>` の比較で
  `f` のみを比較し、後勝ちが安定する形で再現性を持たせる
- **cancel**: `std::atomic<bool> cancel_flag_` を毎反復チェック

## ROS との関係

このアルゴリズムは ROS の型を一切扱わない。`nav_msgs::msg::OccupancyGrid`
から `GridMapView` への変換は呼び出し側（`texnitis_mbf_planners` の
`AStarPlanner` アダプタ、または `MapProvider` 経由）が担う。

## 計算量

- 最悪: O(W × H × log(W × H))
- 実用: 膨張後の自由空間の面積で抑えられることが多い
- `max_iterations=0` のとき自動的に `2 * width * height` を上限に設定

## パラメータ（`AStarParams`）

| フィールド | 既定 | 意味 |
|---|---|---|
| `occupied_threshold` | 65 | これ以上の値で通行不可 |
| `unknown_is_obstacle` | true | `-1` セルを障害物として扱う |
| `inflation_radius` | 0.30 [m] | 膨張半径 |
| `max_iterations` | 0 | 0 で自動。探索の発散を防ぐ |
| `heuristic_weight` | 1.0 | >1 で速度優先、admissible でなくなる |
| `allow_diagonal` | true | 8 近傍探索 |

## サンプルコード

C++:

```cpp
#include <texnitis_nav_core/grid_map_view.hpp>
#include <texnitis_nav_core/planners/astar_planner.hpp>

texnitis::nav_core::AStarParams params;
params.inflation_radius = 0.0;
texnitis::nav_core::AStarPlanner planner (params);

texnitis::nav_core::GridMapView map;
// map.data / width / height / resolution / origin_x / origin_y を埋める

texnitis::nav_core::Path2D path;
auto status = planner.planPath (map, {0.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, path);
```

Python:

```python
import numpy as np
import texnitis_nav_core as nc

grid = np.zeros((10, 10), dtype=np.int8)
view = nc.GridMapView.from_numpy(grid, resolution=0.1)
planner = nc.AStarPlanner(nc.AStarParams())
status, path = planner.plan_path(view, nc.Pose2D(0.05, 0.05, 0.0),
                                       nc.Pose2D(0.85, 0.85, 0.0))
print(status, len(path))
```

## 参考文献

- Hart, P. E.; Nilsson, N. J.; Raphael, B. (1968). *A Formal Basis for the
  Heuristic Determination of Minimum Cost Paths*. IEEE TSSC 4(2): 100–107.
- Daniel, K.; Nash, A.; Koenig, S.; Felner, A. (2010). *Theta\*: Any-Angle
  Path Planning on Grids*. JAIR 39: 533–579.
