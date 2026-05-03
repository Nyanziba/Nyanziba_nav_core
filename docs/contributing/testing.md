# テスト戦略

> このファイルは骨格段階。M0 で fixture を整備した後に最終形へ更新する。

## 三層構造

1. **純粋単体テスト（GoogleTest / pytest）** — 1 回呼び出しの入出力同等性
2. **状態管理シナリオテスト（GoogleTest + Python シミュレータ）** — 複数ステップの状態遷移
3. **シミュレータ E2E（Python）** — 実機相当の disturbance / metrics でゴール走破

## シナリオ fixture

- 場所: `tests/scenarios/<id>/scenario.yaml` と `tests/scenarios/<id>/expected.json`
- ファイルは C++（yaml-cpp / nlohmann_json）と Python（PyYAML / json）の両方から読まれる
- 確率的シナリオ（pose ノイズ等）は seed 固定で 5 試行回し、`metrics.py` の閾値を全て満たすこと
- 一覧は計画書（`docs/contributing/release_process.md`）と root の計画書を参照

## 数値メトリクス（実装後 metrics.py が計測）

- `path_curvature_jump_max`
- `cmd_vel_jerk_max`
- `cmd_vel_rms_at_rest`
- `goal_reach_flip_count`
- `goal_reach_time`
- `collision_count`
- `nan_inf_count`

## ローカル実行

```bash
# C++
cmake -S . -B build -DNAV_CORE_BUILD_TESTS=ON
cmake --build build -j
ctest --test-dir build --output-on-failure

# Python
pip install '.[sim,test]'
MPLBACKEND=Agg pytest
```
