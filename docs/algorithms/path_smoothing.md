# path_postprocessing（予定）

A\* のジグザグ出力を Pure Pursuit / MPPI が滑らかに追従できるよう、
B-spline / line cleaner / velocity profiler を後処理で挟むモジュール。

> 実装は M9 以降のフォローアップで本リポジトリに追加予定。
> 現状は API シェイプ（`include/texnitis_nav_core/path_postprocessing/`）
> の予約ディレクトリのみ存在し、機能は未実装。

設計方針:

- `LineCleaner`: 重複/逆走 pose を除去（Ramer-Douglas-Peucker）
- `BsplineSmoother`: 曲率連続な path に再サンプル
- `VelocityProfiler`: 各セグメントに加速度・jerk 上限を満たす速度を割り当て

シナリオ S-13（曲率連続性）でこの層が機能するかを検証する想定。
