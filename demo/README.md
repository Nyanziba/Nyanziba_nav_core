# texnitis_nav_core デモ

`texnitis_nav_core` を **C++ から / Python から** どう呼ぶか、最小手順を示すデモ集です。
すべて ROS なしで動作します。

```text
demo/
├── cpp/
│   ├── astar_lookahead/   A* で計画 → Lookahead で追従、PPM 画像を出力
│   ├── mppi/              MPPI で経路追従、PPM 画像を出力
│   └── CMakeLists.txt
└── python/
    ├── astar_lookahead.py 同上を matplotlib で可視化
    ├── mppi.py            MPPI を matplotlib で可視化
    ├── simulator_demo.py  Disturbance 入りエピソードを GIF / PNG 保存
    ├── maps/corridor.yaml サンプルマップ（YAML）
    └── output/            描画結果の出力先（gitignore 済）
```

---

## シミュレーションの仕組み

### 1. 状態と運動学

ロボットの状態は 2D pose `(x, y, yaw)` と body 系速度 `(vx, vy, wz)`。
コントローラが返す `Twist2D` を 1 ステップ `dt` で離散積分してロボットを進めます。

| モード | 整合する車種 | 1 ステップ式 |
|---|---|---|
| **unicycle** | 差動駆動 | `x' = x + vx·cos(yaw)·dt`, `y' = y + vx·sin(yaw)·dt`, `yaw' = yaw + wz·dt` (vy は無視) |
| **holonomic** | メカナム / オムニ | body 速度 `(vx, vy)` を yaw で回転して world 系で積分 |
| **mecanum (core)** | メカナム + 動力学 | C++ コア `MecanumKinematics` に委譲。速度上限 / yaw 正規化 / wheel 速度算出まで含む |

Python 側では `texnitis_nav_core.sim.kinematics` モジュールにこの 3 種類を提供しています。
`step_mecanum_via_core` だけは pybind11 経由で **MPPI と同じ離散化式** を使うので、
シミュレータ駆動結果と MPPI 内部 rollout が数値的に一致します。

### 2. シミュレーションループ

`texnitis_nav_core.sim.runner.run_episode` は次を `max_steps` 回繰り返します:

```text
for step in 0..max_steps:
    observed_pose = perturb(true_pose)            # localization ノイズを乗せる
    status, twist = controller.compute_command(observed_pose)
    if any(NaN/Inf in twist): record, twist = 0
    delivered = disturbance.push_cmd(twist)       # cmd 遅延 + 量子化
    true_pose = integrate(true_pose, delivered, dt)
    if occupancy_map.is_blocked(true_pose):       # cell-level collision
        history.collision_steps.append(step)
    reached = controller.is_goal_reached(...)
    history.append(...)
    if reached: break
```

ポイント:

- **観測 pose と真 pose を分離**: `Disturbance` で観測値だけにノイズを乗せます。
  controller は観測 pose しか見えず、真 pose は衝突判定 / 描画にだけ使います。
- **cmd_vel 遅延**: `cmd_vel_latency_steps` 個の deque に push して、過去の cmd を
  実機に届くタイミングで pop します（CAN/USB 系の遅延を再現）。
- **量子化**: `cmd_vel_quant_linear / angular` で round-to-step。実機の CAN ステップ近似。
- **衝突判定**: 占有グリッドのセル参照（しきい値以上または OOB / unknown）。
  `radius` を考慮しない最小実装です（拡張は M9 以降）。

### 3. 描画

Python: `texnitis_nav_core.sim.visualize.render_episode` が matplotlib で次を 1 枚に描画:

- 占有グリッド（薄いグレー）
- 計画経路（青破線）
- 実走行軌跡（緑実線）
- 開始位置（緑丸）/ 終了位置（赤星）

`save_png=...` で PNG 保存、`show=True` でインタラクティブ表示、
GIF アニメは `simulator_demo.py` がフレームを連番 PNG として吐いて
`imageio` または `matplotlib.animation` で結合します。

C++: matplotlib が無いので **PPM (Portable Pixmap, ASCII P3)** で直接描画します。
PPM は `convert trajectory.ppm trajectory.png` (ImageMagick) や macOS の Preview.app で
そのまま開けるので、依存ゼロでビルド・確認できます。

### 4. メトリクス

`History` に詰まった時系列から `texnitis_nav_core.sim` が次を計算:

| メトリクス | 役割 |
|---|---|
| `goal_reach_flip_count` | stateful フラグの False→True 遷移数。1 が正常 |
| `cmd_vel_jerk_max` | 急激な cmd_vel 変化を検知 |
| `cmd_vel_rms_at_rest` | 停止後の cmd_vel chatter（localization ノイズの振動） |
| `path_curvature_jump_max` | 経路の隣接セグメント曲率変化のピーク |
| `collision_count` | 占有セルを踏んだ回数 |
| `nan_inf_count` | NaN / Inf が出た cmd の数 |

これらは `tests/scenarios/` の YAML から閾値で assert され、退行を CI で検出します。

---

## 動かし方

### C++ デモ

```bash
cd demo/cpp
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/texnitis_nav_core/install
cmake --build build -j
./build/astar_lookahead     # → trajectory.ppm を生成
./build/mppi                # → mppi_trajectory.ppm を生成
```

詳細は [cpp/astar_lookahead/README.md](cpp/astar_lookahead/README.md) と
[cpp/mppi/README.md](cpp/mppi/README.md)。

### Python デモ

```bash
# nav_core の wheel を含むよう pip install しておく
pip install '..[sim]'   # demo/ から見たときのリポ root
cd demo/python

python astar_lookahead.py            # → output/astar_lookahead.png
python mppi.py                       # → output/mppi.png
python simulator_demo.py --gif       # → output/episode.gif
python simulator_demo.py --show      # インタラクティブ
```

詳細はそれぞれのスクリプト先頭の docstring を参照。

---

## 関連ドキュメント

- `docs/usage/simulator_guide.md` — シミュレータ API リファレンス
- `docs/usage/quickstart_cpp.md` — C++ 利用の最小例
- `docs/usage/quickstart_python.md` — Python 利用の最小例
- `docs/algorithms/` — 各アルゴリズムの理論
