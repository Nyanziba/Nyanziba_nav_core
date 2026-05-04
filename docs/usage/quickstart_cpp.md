# C++ クイックスタート

ROS なしで `texnitis_nav_core` を C++ から直接使う最小手順。

## 必要なもの

- CMake 3.20 以降
- C++20 コンパイラ（GCC 11+ / Apple Clang 15+）
- Eigen3（MPPI を使う場合のみ）
  - Ubuntu 24.04: `sudo apt install libeigen3-dev`
  - macOS: `brew install eigen`

## ビルド

```bash
git clone https://github.com/Nyanziba/texnitis_nav_core.git
cd texnitis_nav_core
cmake -S . -B build -DNAV_CORE_BUILD_TESTS=ON -DNAV_CORE_WITH_MPPI=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## 最小サンプル: A\* と Lookahead

```cpp
#include <texnitis_nav_core/grid_map_view.hpp>
#include <texnitis_nav_core/planners/astar_planner.hpp>
#include <texnitis_nav_core/controllers/lookahead_controller.hpp>

#include <vector>
#include <cstdint>

int main () {
    using namespace texnitis::nav_core;

    std::vector<int8_t> grid (100, 0);
    GridMapView m;
    m.data       = grid.data ();
    m.width      = 10;
    m.height     = 10;
    m.resolution = 0.1;

    AStarParams aparams;
    aparams.inflation_radius = 0.0;
    AStarPlanner planner (aparams);
    Path2D path;
    auto status = planner.planPath (m, {0.05, 0.05, 0.0}, {0.85, 0.85, 0.0}, path);
    if (status != PlanResult::Success) return 1;

    LookaheadParams cparams;
    cparams.use_diff_drive = false;
    LookaheadController controller (cparams);
    controller.setPlan (path);

    Twist2D cmd;
    controller.computeCommand ({0.0, 0.0, 0.0}, cmd);
    return 0;
}
```

## downstream プロジェクトへの組み込み

```cmake
find_package(texnitis_nav_core REQUIRED CONFIG)
target_link_libraries(my_target PRIVATE texnitis_nav_core::texnitis_nav_core)
```

## トラブルシュート

- **`Could not find a package configuration file provided by "Eigen3"`**:
  Eigen3 がインストールされていない。上記の OS 別コマンドで入れる。
  または `-DNAV_CORE_WITH_MPPI=OFF` で MPPI を無効化。
- **`legacy_harness` が見つからない**: ROS 2 Jazzy 環境を source する
  必要がある（`source /opt/ros/jazzy/setup.bash` または pixi global の
  `~/.pixi/envs/default/setup.zsh`）。
