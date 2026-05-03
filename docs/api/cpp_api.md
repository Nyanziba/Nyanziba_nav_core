# C++ API リファレンス（入口）

## 自動生成 API ドキュメント

C++ 公開 API は Doxygen で生成する。生成物は `docs/api/cpp/html/index.html` に出力される（CI で
`doxygen doxygen/Doxyfile` を実行し、リリースタグで GitHub Pages にデプロイ）。

```bash
# ローカル生成
cd /path/to/texnitis_nav_core
doxygen doxygen/Doxyfile
open docs/api/cpp/html/index.html  # macOS
xdg-open docs/api/cpp/html/index.html  # Linux
```

## 主要ヘッダの読む順序

詳細は [../reading_guide.md](../reading_guide.md) を参照。

| ヘッダ | 役割 |
|---|---|
| `texnitis_nav_core/types.hpp` | 共通 POD 型 |
| `texnitis_nav_core/grid_map_view.hpp` | 占有グリッドの zero-copy ビュー |
| `texnitis_nav_core/result.hpp` | エラーコード enum |
| `texnitis_nav_core/planner_base.hpp` | プランナーの純粋仮想 IF |
| `texnitis_nav_core/controller_base.hpp` | コントローラの純粋仮想 IF |
| `texnitis_nav_core/goal_checker.hpp` | XY/yaw 到達判定（stateful） |

## 公開ターゲット

下流リポからは `find_package(texnitis_nav_core CONFIG)` で次のターゲットを使う:

- `texnitis_nav_core::texnitis_nav_core` — メインライブラリ

ターゲット名と export 設定は M1 で確定する。
