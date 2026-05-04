# texnitis_nav_core

ROS から独立した C++ ナビゲーションコア。A\* / Pure Pursuit / MPPI を含むアルゴリズム
本体と、その Python バインディング、純 Python の 2D シミュレータを提供する。

## 目的

- ROS 2 / ROS 1 / 単独 C++ アプリ / Python のいずれからも同じアルゴリズムを再利用する
- アルゴリズム本体を `rclcpp` から切り離し、ユニットテストとシミュレータで動作を保証する
- `move_base_flex` 互換の薄いプラグインを別リポ（[`texnitis_mbf_plugins`](https://github.com/Nyanziba/texnitis_mbf_plugins)）から呼べる形に保つ

## 構成

```text
include/texnitis_nav_core/  pure C++ headers (Pose2D, GridMapView, Planner/Controller base, ...)
src/                         implementations
bindings/python/             pybind11 module
python/texnitis_nav_core/    Python package + 2D simulator (sim/)
tests/                       GoogleTest unit / scenario tests
tests_python/                pytest tests (bindings + simulator)
scenarios/                   YAML scenarios shared between C++ and Python tests
demo/                        ready-to-run C++ / Python usage demos
docs/                        architecture / design rationale / algorithms / usage / contributing
```

## クイックリンク

| やりたいこと | 入口 |
|---|---|
| とりあえず動かす | [demo/README.md](demo/README.md) |
| C++ から最小サンプル | [docs/usage/quickstart_cpp.md](docs/usage/quickstart_cpp.md) |
| Python から最小サンプル | [docs/usage/quickstart_python.md](docs/usage/quickstart_python.md) |
| シミュレータの使い方 | [docs/usage/simulator_guide.md](docs/usage/simulator_guide.md) |
| **move_base_flex と連携する** | [docs/usage/move_base_flex_integration.md](docs/usage/move_base_flex_integration.md) |
| アルゴリズムの理論 | [docs/algorithms/](docs/algorithms/) |
| パラメータ全リファレンス | [docs/usage/parameters.md](docs/usage/parameters.md) |
| 設計判断と根拠 | [docs/design_rationale.md](docs/design_rationale.md) |

## ビルド（C++ コアのみ、ROS 不要）

```bash
cmake -S . -B build -DNAV_CORE_BUILD_TESTS=ON -DNAV_CORE_WITH_MPPI=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Eigen3 が必要（MPPI=ON の場合のみ）:

- Ubuntu 24.04: `sudo apt install libeigen3-dev`
- macOS: `brew install eigen`

## インストール（Python バインディング + シミュレータ）

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install --upgrade pip
pip install '.[sim,test]'
pytest
```

## デモ

`demo/` 配下に C++ と Python の使い方サンプルがあります:

```bash
# C++: A* + Lookahead で経路追従、PPM 画像を出力
cmake --install build --prefix /tmp/nav_core_install
cd demo/cpp && cmake -S . -B build -DCMAKE_PREFIX_PATH=/tmp/nav_core_install
cmake --build build -j
./build/astar_lookahead    # → trajectory.ppm
./build/mppi               # → mppi_trajectory.ppm

# Python: matplotlib で描画、シミュレータの GIF 生成も
cd demo/python
python astar_lookahead.py
python mppi.py
python simulator_demo.py --gif
```

詳細は [demo/README.md](demo/README.md) を参照。

## move_base_flex との連携

ROS 2 Jazzy の `move_base_flex` から本コアを利用する場合は、
[`texnitis_mbf_plugins`](https://github.com/Nyanziba/texnitis_mbf_plugins) リポ
にあるプラグインを使います。最小手順:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Nyanziba/texnitis_nav_core.git
git clone https://github.com/Nyanziba/texnitis_mbf_plugins.git
vcs import < texnitis_mbf_plugins/third_party/move_base_flex.repos
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to texnitis_mbf_bringup
source install/setup.bash
ros2 launch texnitis_mbf_bringup texnitis_mbf.launch.py
```

設計や YAML マッピング、デバッグ手順を含む完全ガイドは
[docs/usage/move_base_flex_integration.md](docs/usage/move_base_flex_integration.md)。

## ROS 2 環境の準備

`legacy_harness` や `texnitis_mbf_plugins` をビルドするには ROS 2 Jazzy が必要です:

- **Linux (Ubuntu 24.04)**: `source /opt/ros/jazzy/setup.bash`
- **macOS (pixi global)**: `source ~/.pixi/envs/default/setup.zsh`
  （pixi-global.toml の `default` env に `ros-jazzy-desktop` が入っている前提）

## ライセンス

MIT License — `LICENSE` を参照。
