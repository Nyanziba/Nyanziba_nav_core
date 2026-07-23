# 設計判断と根拠

判断は「**問い → 検討した代替案 → 採用した案と理由**」の形で記録する。
後から読む人が結論だけでなく、なぜ他案を捨てたのかを追えるようにする。

---

## なぜ ROS 2 から切り離してコアを別リポにするのか

- **検討した代替案**:
  - (A) 既存 `texnitis_move_base_like` パッケージのまま `move_base_flex` に対応
  - (B) 1 リポに ROS 層と非 ROS 層を共存、CMake オプションで切替
  - (C) ★採用：コア（純 C++）と mbf プラグイン（ROS 2）を **別 GitHub リポ** に分割
- **採用理由**:
  - **ROS 世代切替への耐性**: ROS 2 の major version が変わっても、
    コアアルゴリズムには影響しない。`Eigen3` だけ揃えばよい。
  - **テスト容易性**: ROS 不在の macOS でも `ctest` が走る。
    GitHub Actions matrix が ubuntu-24.04 / macos-latest の両方で
    同じテストを通せる（M2 / M4 / M5 で実証済）。
  - **再利用性**: pybind11 経由で Python から、将来的には ROS 1 や
    別のミドルウェア（gRPC, ZeroMQ 等）からも同じアルゴリズムを呼べる。
  - **CI のシンプル化**: Eigen を含む lightweight なジョブと、
    osrf/ros:jazzy-desktop が要る legacy_harness ジョブを分離できる。

## なぜ pluginlib アダプタを「薄く」するのか

- **検討した代替案**:
  - (A) 旧コードのように `rclcpp::Node` をプラグイン内部に保持し、
        パラメータも `declare_parameter` で散らばらせる
  - (B) ★採用：プラグインは `Params` 構造体充填と型変換のみ。
        アルゴリズムはコアに委譲
- **採用理由**:
  - **テスト**: アルゴリズムを ROS なしで GoogleTest にかけられる
  - **再利用**: 同じコアを Python シミュレータからも叩ける
  - **デバッグ**: ROS の trace と algorithm の trace を分離できる
  - **重複削減**: `worldToMap` や inflation を 1 箇所で書けば、
    座標変換やinflationの重複実装を1箇所に集約できた

## なぜ `MapProvider` をシングルトンにするのか

- **問題**: mbf ros2 ブランチに `mbf_costmap_core` 相当が無く、
  プラグインが個別に /map を購読すると帯域・メモリの無駄
- **検討した代替案**:
  - (A) 各プラグインが独自に購読
  - (B) ★採用：`MapProvider::get(node)` で node 単位の唯一インスタンスを
        `weak_ptr` テーブルで管理
- **採用理由**:
  - mbf_simple_nav は同一 node に複数プラグインをロードする運用が前提
    なので、node ポインタをキーにすれば共有できる
  - node 破棄で `weak_ptr` が expire し、エントリは自動的にクリア
  - reliable + transient_local QoS で、後発のプラグインも最新マップを
    取得できる

## なぜ pybind11 + scikit-build-core なのか

- **検討した代替案**:
  - (A) Cython（学習コスト・C++20 対応の弱さ）
  - (B) ctypes / FFI（型情報が落ちる、numpy 連携が難しい）
  - (C) ★採用：pybind11 + scikit-build-core
- **採用理由**:
  - **STL 自動変換**: `std::vector<Pose2D>` のような型がそのまま動く
  - **numpy 連携**: `py::array_t<int8_t>` で zero-copy バッファを借用、
    `py::keep_alive` で寿命を結べる
  - **CMake 統合**: scikit-build-core で `pip install .` から CMake を
    呼べる。CI で wheel ビルドまで通せる（M7 で実証済）

## なぜ CasADi はオプション（`NAV_CORE_WITH_MPPI`）なのか

- 組込み Linux など CasADi が入らない環境でも A\* / Pure Pursuit を
  使えるようにする
- MPPI は Eigen のみで動くサンプリングベース実装を提供。CasADi で
  symbolic dynamics を高速化する経路は M9 以降のフォローアップで導入
- CI では ON / OFF の両方を常に通す（matrix で 2 ジョブ）

## なぜ Python 側にシミュレータを同梱するのか

- ROS なしで「アルゴリズムが実機相当の条件で動くか」を確認する手段が
  なかった
- `Disturbance` で localization 震え・制御遅延・cmd_vel 量子化を再現
- pytest マーカー `sim` で実行できるので CI に組み込みやすい
- `MecanumKinematics` を介して C++ コアと同じ運動モデルを使えるので、
  Python 側で別実装を持たない（数値乖離を防ぐ）

## なぜ「テスト fixture を最初に作る」のか

- リファクタの最大リスクは「ゴール再到達時に状態管理が壊れる」など、
  単発テストでは捕まらない振る舞い退行
- `tests/scenarios/S-01..S-17` を YAML で先に固定することで、新コアを
  書いた時点で機械的に退行を検出できる
- legacy_harness で旧コードを駆動して `expected.json` を採取する設計
  にしたので、フィクスチャ自体は ROS が要る一方、新コアの単体テストは
  ROS 無しで CI が走る
