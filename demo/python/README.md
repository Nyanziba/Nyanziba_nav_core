# Python デモ

`pip install '.[sim]'` 済みであることを前提とします（リポ root から見て
`pip install '.[sim]'`、`demo/python/` から見ると `pip install '../..[sim]'`）。

## 1. A\* + Lookahead

```bash
python astar_lookahead.py            # output/astar_lookahead.png 保存
python astar_lookahead.py --show     # GUI 表示
python astar_lookahead.py --start 0.3 0.3 0 --goal 4.7 0.5 0
```

`maps/corridor.yaml` を読み込み、A\* で計画して Lookahead で追従します。

## 2. MPPI

```bash
python mppi.py
python mppi.py --horizon 30 --num-samples 512
```

直線経路を MPPI で追従します。MPPI は Eigen-only なので `texnitis_nav_core`
が `NAV_CORE_WITH_MPPI=ON` でビルドされている必要があります。

## 3. シミュレータ（フル機能）

`run_episode` を直接呼ぶデモで、disturbance とメトリクスも測定します。

```bash
python simulator_demo.py                    # PNG 保存
python simulator_demo.py --gif              # GIF アニメ保存
python simulator_demo.py --show             # 対話表示
python simulator_demo.py --noise 0.05       # localization ノイズ
python simulator_demo.py --latency 2        # cmd_vel 2 step 遅延
```

出力例:

```text
planned 51 poses
reach_step          = 168
goal_reach_flips    = 1
cmd_vel_jerk_max    = 4.823
cmd_vel_rms_at_rest = 0.0008
collisions          = 0
NaN/Inf             = 0
wrote output/episode.gif
```

## maps/

`corridor.yaml` は ASCII で 50 x 30 セルのコの字マップを定義しています。
書式:

```yaml
resolution: 0.10
origin: [0.0, 0.0]
occupancy: |
  ..........
  ..######..
  ..........
```

`#` がブロック、`.` が自由空間、`?` が unknown (`-1`)。

## トラブルシュート

- **`MPPI bindings not built`**: `NAV_CORE_WITH_MPPI=ON` で再ビルドし
  `pip install --force-reinstall '.[sim]'`。
- **GIF が作れない**: pillow / imageio が要ります。`pip install pillow`。
- **macOS で matplotlib が hang する**: `MPLBACKEND=Agg` を設定。
  `--show` を使わなければ自動で Agg にします。
