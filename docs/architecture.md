# アーキテクチャ概観

## レイヤ

```mermaid
graph TD
  subgraph "texnitis_nav_core (このリポ)"
    types[types.hpp / GridMapView]
    iface[PlannerBase / ControllerBase / MppiControllerBase]
    impl[A* / HeightAwareAStar / Lookahead / DiffDrivePP / MPPI]
    pp[path_postprocessing]
    py[pybind11 module _core]
    sim[Python sim package]
    types --> iface
    iface --> impl
    impl --> pp
    impl --> py
    py --> sim
  end

  subgraph "texnitis_mbf_plugins (別リポ・ROS 2 + mbf)"
    adapter[SimplePlanner / SimpleController adapters]
    mp[MapProvider]
    bringup[bringup launch / config]
  end

  iface -.PRIVATE link.-> adapter
  adapter --> mp
  bringup --> adapter
```

## 依存方向

- `texnitis_nav_core` は **rclcpp / nav_msgs / geometry_msgs / pluginlib に一切依存しない**
- 外部ヘビー依存は Eigen3（必須）と CasADi（`NAV_CORE_WITH_MPPI=ON` のときのみ）
- pybind11 は `NAV_CORE_BUILD_PYTHON=ON` のときのみ
- 下流（`texnitis_mbf_plugins`）はこのコアを **PRIVATE link**。ROS 依存型はアダプタ層に閉じ込める

## ディレクトリの責務

| ディレクトリ | 責務 |
|---|---|
| `include/texnitis_nav_core/` | 公開ヘッダ。POD 型 (`Pose2D`, `Twist2D`, `Path2D`) と純粋仮想インターフェース |
| `src/` | 実装本体 |
| `bindings/python/` | pybind11 で `_core` モジュールを生成 |
| `python/texnitis_nav_core/` | Python パッケージ。`sim/` 配下に純 Python の 2D シミュレータ |
| `tests/` | GoogleTest 単体・シナリオテスト |
| `tests_python/` | pytest（バインディングとシミュレータ） |
| `scenarios/` | シナリオ YAML（C++ と Python の両方から読まれる） |
| `examples/` | C++ / Python のデモコード |

## このページの位置付け

- **全体像** はここで。
- **なぜこの分け方なのか** は [design_rationale.md](design_rationale.md)。
- **ソースの読む順序** は [reading_guide.md](reading_guide.md)。
