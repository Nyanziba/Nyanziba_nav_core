# C++ デモ: A\* + Lookahead

A\* で経路を計画し、`LookaheadController` で追従するシンプルな例です。

## 何をするコードか

- 50 × 30 セル（5 m × 3 m, 解像度 0.10 m）の占有グリッドにコの字の障害物を置く
- `(0.5, 0.5)` から `(4.5, 2.5)` まで A\* で経路を計画
- `LookaheadController`（ホロノミック）で追従、`dt = 0.05 s` で離散積分
- 経路と軌跡を `trajectory.ppm` に書き出す

## ビルド & 実行

`texnitis_nav_core` をインストール済みのことを前提に:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/install
cmake --build build -j
./build/astar_lookahead
```

期待される出力:

```text
planned 49 poses
goal reached at step 87
wrote trajectory.ppm (88 steps)
```

`trajectory.ppm` を Preview.app などで開くか、ImageMagick で変換:

```bash
convert trajectory.ppm trajectory.png
```

凡例:

| 色 | 意味 |
|---|---|
| 黒 | 障害物 |
| 白 | 自由空間 |
| 青点 | 計画経路 |
| 緑線 | 実走行軌跡 |
| 緑円 | 開始位置 |
| 赤円 | 終了位置 |

## コードを読むときの読み順

1. `main()` 冒頭の `makeMap()` と `GridMapView` セットアップ
2. `AStarPlanner` の構築 + `planPath`
3. `LookaheadController` の構築 + `setPlan` + 制御ループ
4. `stepUnicycle` ／ `pose.x += ...` の離散積分
5. `savePpm` の描画ロジック
