# Height-aware A\*

`texnitis::nav_core::HeightAwareAStarPlanner` は、占有グリッドに加えて
**高さグリッド**を考慮できる A\* プランナー。M4 の最小実装としては、
高さしきい値超えセルを lethal にマージしてから通常の A\* を走らせる
シンプルな wrapper になっている（heading-bin / yaw smoothing は M9
以降の発展に譲る）。

## 入出力

- 入力: `GridMapView`（占有）+ `HeightProvider` 経由の `HeightGridView`
- 出力: `Path2D` または `PlanResult` の失敗値

## アルゴリズム要点

1. `HeightProvider::getLatest` で高さグリッドを借りる
2. `require_height_grid=true` で未到着なら `NotInitialized` を即返す
3. 占有グリッドをコピーした `merged_` バッファを作る
4. 各セルで `mapToWorld` → 高さグリッドの対応セルを確認、
   高さが `height_lethal_threshold` 以上なら 100 (lethal) に上書き
5. その合成マップで内部 `AStarPlanner::planPath` を呼ぶ

## パラメータ（`HeightAwareAStarParams`）

| フィールド | 既定 | 意味 |
|---|---|---|
| `astar` | — | 共通 A\* パラメータ（inflation など） |
| `height_lethal_threshold` | 50 | これ以上の高さ値で通行不可 |
| `require_height_grid` | true | 未到着なら NotInitialized で失敗 |

## サンプル

```cpp
texnitis::nav_core::HeightAwareAStarPlanner planner (params, provider);
texnitis::nav_core::Path2D path;
auto status = planner.planPath (map, start, goal, path);
```
