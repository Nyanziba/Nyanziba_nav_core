# texnitis_nav_core

ROS から独立した C++ ナビゲーションコア。A\* / Pure Pursuit / MPPI を含むアルゴリズム本体と、その Python バインディング、純 Python の 2D シミュレータを提供する。

> このリポジトリは現時点では **骨格（scaffold）のみ** です。実装は `docs/architecture.md` と
> `docs/contributing/release_process.md` のマイルストーン順に追加していきます。

## 目的

- ROS 2 / ROS 1 / 単独 C++ アプリ / Python のいずれからも同じアルゴリズムを再利用する
- アルゴリズム本体を `rclcpp` から切り離し、ユニットテストとシミュレータで動作を保証する
- `move_base_flex` 互換の薄いプラグインを別リポ（`texnitis_mbf_plugins`）から呼べる形に保つ

## 構成

```text
include/texnitis_nav_core/  pure C++ headers (Pose2D, GridMapView, Planner/Controller base, ...)
src/                         implementations
bindings/python/             pybind11 module
python/texnitis_nav_core/    Python package + 2D simulator (sim/)
tests/                       GoogleTest unit / scenario tests
tests_python/                pytest tests (bindings + simulator)
scenarios/                   YAML scenarios shared between C++ and Python tests
docs/                        architecture / design rationale / algorithms / usage / contributing
```

## ビルド（C++ コアのみ、ROS 不要）

```bash
cmake -S . -B build -DNAV_CORE_BUILD_TESTS=ON -DNAV_CORE_WITH_MPPI=OFF
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## ビルド（legacy_harness 等で ROS 2 Jazzy が必要なとき）

- **Linux (Ubuntu 24.04)**: `source /opt/ros/jazzy/setup.bash`
- **macOS (pixi global)**: `source ~/.pixi/envs/default/setup.zsh`
  （pixi-global.toml の `default` env に `ros-jazzy-desktop` が入っている前提）

その後 colcon workspace で `texnitis_move_base_like` をビルドし、
`tests/legacy_harness/CMakeLists.txt` を `cmake -S tests/legacy_harness -B
build/legacy_harness` で構成する。詳細は [tests/legacy_harness/README.md](tests/legacy_harness/README.md) を参照。

## インストール（Python バインディング + シミュレータ）

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install --upgrade pip
pip install '.[sim,test]'
pytest
```

## ドキュメント

- 設計と判断根拠: `docs/design_rationale.md`
- アルゴリズム解説: `docs/algorithms/`
- API リファレンス: `docs/api/cpp_api.md` / `docs/api/python_api.md`
- 使い方: `docs/usage/quickstart_cpp.md` / `docs/usage/quickstart_python.md` / `docs/usage/simulator_guide.md`
- コードリーディング: `docs/reading_guide.md`

## ライセンス

MIT License — `LICENSE` を参照。
