# Mecanum MPPI コントローラ

`texnitis::nav_core::mppi::MecanumMppiController` はサンプリングベースの
MPPI（Model Predictive Path Integral）。Eigen のみで動く軽量実装。

## アルゴリズム要点

1. ノミナル制御列 `u_nominal[horizon]` を保持
2. 各サンプル `i` で `u_i = u_nominal + epsilon`、`epsilon` は Gaussian(0, sigma)
3. 各サンプルを `MecanumKinematics::updateState` で `horizon` ステップ展開
4. ロールアウトコスト = w_track * 経路追従距離² + w_yaw * yaw誤差² + w_control * |u|²
   + 5 * w_track * 終点距離²
5. softmax 重み: `w_i = exp(-(cost_i - min_cost) / lambda)`
6. 加重平均で `u_nominal` を更新、最初のステップを実機 cmd に反映
7. 残りを 1 step shift して warm-start

## CasADi-backed 評価器

M5 時点では Eigen 単独。CasADi をリンクしたとき symbolic dynamics で
`updateState` を高速化する経路は M9 のフォローアップで導入予定。
仕掛けは `TEXNITIS_NAV_CORE_WITH_MPPI` の追加マクロ枠を残してある。

## 主なパラメータ（`MppiParams`）

| フィールド | 既定 | 意味 |
|---|---|---|
| `horizon` | 25 | 予測ホライズン |
| `num_samples` | 256 | サンプル数 |
| `lambda` | 0.10 | softmax 温度 |
| `sigma` | (0.30, 0.30, 0.40) | (ax, ay, alpha) のノイズ stddev |
| `u_max` | (1.50, 1.50, 1.50) | 制御絶対値の上限 |
| `dt` | 0.05 | 離散時間ステップ |
| `w_track` / `w_yaw` / `w_control` | 1.0 / 0.30 / 0.05 | コスト重み |
| `v_max` / `omega_max` / `wheel_base` | 0.50 / 1.50 / 0.30 | 機体パラメータ |
| `seed` | 42 | 乱数 seed（再現性） |

## 参考文献

- Williams, G.; Drews, P.; Goldfain, B.; Rehg, J. M.; Theodorou, E. A. (2018).
  *Information Theoretic MPC for Model-Based Reinforcement Learning*.
  ICRA 2018.
