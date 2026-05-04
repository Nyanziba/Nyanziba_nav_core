# C++ デモ: MPPI

Eigen-only の MPPI（Model Predictive Path Integral）でメカナム車を
直線経路に追従させるデモです。

## 何をするコードか

- 60 × 24 セル（6 m × 2.4 m）の空きグリッド
- `(0.5, 0.5) → (5.5, 1.5)` の直線経路を生成
- `MecanumMppiController`（horizon=20, num_samples=256）で追従
- 軌跡を `mppi_trajectory.ppm` に書き出す

## ビルド & 実行

`texnitis_nav_core` を `NAV_CORE_WITH_MPPI=ON` でインストール済みのことを前提に:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/install
cmake --build build -j
./build/mppi
```

期待される出力:

```text
goal reached at step 78
wrote mppi_trajectory.ppm (79 steps)
```

## なぜ MPPI のテストが直線か

- MPPI のサンプリングは確率的で、複雑な経路だとデモが flaky になります。
- 直線追従なら 256 サンプルでも十分収束し、CI で確実に動きます。
- 実機向けの曲線追従テストは `tests/scenarios/S-10` (MPPI 再現性) で
  カバーされます。

## パラメータを変えてみる

MPPI の挙動は `MppiParams` で調整できます。代表的な変更:

| 変えると起こること | パラメータ |
|---|---|
| サンプリング探索範囲を広げる | `params.sigma << 0.5, 0.5, 0.5` |
| 経路追従を強める | `params.w_track = 5.0` |
| 制御をスムーズにする | `params.w_control = 0.5` |
| ゴール到達判定を緩める | `params.goal_checker.xy_tolerance = 0.20` |
