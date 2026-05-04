# tests/legacy_harness/

旧 `texnitis_move_base_like` の C++ プラグインを Python から駆動して、`tests/scenarios/<id>/expected.json` を **正解値として固定する** ためのハーネス設計と実装を置く場所。

> このディレクトリはまだ設計書のみ。実装ファイル (`run_legacy.py`, `legacy_runner.py`) は M0 後半で追加する。

---

## 何を解決するためのコードか

- 新 `texnitis_nav_core` を書く前に、**旧コードが現に出力している pose / twist / outcome 列を JSON に固定する**
- 新コアは、同じ `scenario.yaml` を再生して `expected.json` と一致することを単体テストで検証する
- これによりリファクタの「目に見えない退行」（goal_stateful の取り違え、加速度履歴のリセット漏れ、cancel フラグの再アクティブ漏れ等）を **機械的に** 防ぐ

---

## 旧コードを起動せず Python から呼ぶ手段の選択

### 検討した代替案

| 案 | 概要 | 問題点 |
|---|---|---|
| (A) 旧 ROS ノードをそのまま起動し、シナリオを Python ROS クライアントから流す | 一番改変が少ない | rclpy + rosbridge + 実時間が絡みテストが flaky になる、CI が重い、TF や tf2_ros のセットアップが必要 |
| (B) 旧 C++ プラグインを ament でビルドし、CTest 駆動の純 C++ harness を書いて expected.json を吐く | ROS は要るがランタイムは決定的 | C++ で YAML / JSON を扱うコストと、Python シミュレータと別の運動学を保つコスト |
| (C) ★採用：**旧 C++ プラグインだけを共有ライブラリとして抜き出し、軽量 fake `rclcpp::Node` 越しに Python から ctypes / pybind11 で叩く** | 旧コード本体を改変しない | fake Node の整備が必要 |

### 採用案 (C) の中身

1. 旧プラグインのソースを **改変せず** に新しい CMake target としてビルドする `tests/legacy_harness/CMakeLists.txt` を用意
2. fake Node 実装 `tests/legacy_harness/fake_node/` を C++ で書く
   - `rclcpp::Node` の代わりに、パラメータ宣言 / 取得とロガーだけを実装する最小スタブ
   - publisher / subscription / TF buffer は使う関数だけスタブ実装（visualization 系はノーオペ）
   - 旧プラグイン側は `rclcpp::Node::SharedPtr` を期待するので、本物の `rclcpp::Node` を最小構成で起動するパターンも用意（rclcpp::init で十分）
3. C ABI ラッパ `tests/legacy_harness/c_api.cpp` で Python から呼べる関数を提供:
   - `legacy_load_planner(name) -> handle`
   - `legacy_planner_plan(handle, map_buf, w, h, res, ox, oy, sx, sy, syaw, gx, gy, gyaw) -> path_buf, status`
   - `legacy_load_controller(name) -> handle`
   - `legacy_controller_set_path(handle, path_buf, ...)`, `legacy_controller_compute(handle, pose, twist_out)`, `legacy_controller_is_goal_reached`, `legacy_controller_cancel`
4. Python 側 `legacy_runner.py` は `ctypes` でこの C ABI を読み、`run_legacy.py` がシナリオ YAML を読んで再生し、`expected.json` を書き出す

### 採用理由（要点）

- 実時間ループや TF tree を絡めずに済む
- ハーネス自体が新コアのテストハーネスと同じ Python シミュレータ実装を流用できる（運動学は `scenarios/<id>/scenario.yaml` の `disturbance` 設定通りに進める純 Python 実装で揃える）
- CI で 1 ジョブとして決定的に走る

---

## ファイル構成（M0 で追加するもの）

```text
tests/legacy_harness/
├── README.md                  ← このファイル
├── CMakeLists.txt             ← legacy_harness を独立 target としてビルド
├── fake_node/
│   ├── fake_node.hpp          ← 最小 rclcpp::Node 互換 helper
│   └── fake_node.cpp
├── c_api.hpp                  ← extern "C" インタフェース
├── c_api.cpp
├── legacy_runner.py           ← ctypes wrapper + scenario YAML 再生エンジン
├── run_legacy.py              ← CLI: `python -m tests.legacy_harness.run_legacy --scenarios tests/scenarios`
└── tests/
    └── test_smoke.py          ← ハーネス自体のスモークテスト（A* で 1 ゴール走破）
```

`run_legacy.py` の責務:

1. 旧リポ（`/Users/nyanziba/hobby/texnitis/R2_packages/navigation/texnitis_move_base_like`）への絶対パスを引数で受け、ビルド済 .so を読む
2. 各 `tests/scenarios/<id>/scenario.yaml` を再生
3. 結果を `tests/scenarios/<id>/expected.json` に書き出す
4. 既に存在する `expected.json` と diff があれば exit 1（CI が "fixture が drift している" として弾く）

---

## fixture を「事実化」する作業の流れ

### 0. ROS 2 Jazzy 環境の用意

- **Linux (Ubuntu 24.04)**: `source /opt/ros/jazzy/setup.bash`
- **macOS (pixi global)**: `source ~/.pixi/envs/default/setup.zsh`

### 1〜5

1. 旧リポをビルド: `colcon build --packages-select texnitis_move_base_like`
2. ハーネスをビルド: `cmake -S tests/legacy_harness -B build/legacy_harness && cmake --build build/legacy_harness`
3. 全シナリオを記録: `python -m tests.legacy_harness.run_legacy --scenarios tests/scenarios --record`
4. 生成された `expected.json` を `git add` してコミット
5. CI 上では `--check` モードで実行し、再現性が壊れたら fail

---

## 注意点

- 旧 `mecanum_mppi_plugin` は CasADi に依存するので、ハーネス側でも CasADi 有無で skip できるようにする（`@pytest.mark.skipif(not casadi_available, ...)`）
- HeightAware 系は `create_subscription` で高さグリッドを取りに行くので、fake Node 側で「subscribe したら即座にシナリオ YAML の `height_map` を渡す」スタブを用意する
- 旧コード側の決定論性: A\* のタイブレークを再現するため、シナリオ YAML 側の `seed` をプラグインに伝える必要はない（A\* は乱数を使わない）。MPPI は seed を渡す
- 旧コードを変更してはいけない（fixture を新コアの正解として固定するため、旧コードに修正を入れるとそもそもの基準が動く）。バグがあっても fixture 採取後に新コアで直す形にする

---

## 関連

- スキーマ: [../scenarios/SCHEMA.md](../scenarios/SCHEMA.md)
- シナリオ一覧: [../scenarios/README.md](../scenarios/README.md)
- 設計判断: [../../docs/design_rationale.md](../../docs/design_rationale.md) の「テスト fixture を最初に作る」節
- テスト戦略: [../../docs/contributing/testing.md](../../docs/contributing/testing.md)
