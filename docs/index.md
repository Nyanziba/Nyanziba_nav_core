# texnitis_nav_core ドキュメント

ROS から独立した C++ ナビゲーションコア。A\* / Pure Pursuit / MPPI を C++ から、Python から、`move_base_flex` プラグインから同じ実装で呼べるようにすることを目的とする。

## 読み始め

| 目的 | まず読むページ |
|---|---|
| 全体像をつかむ | [architecture.md](architecture.md) |
| なぜこの設計なのかを知る | [design_rationale.md](design_rationale.md) |
| ソースを読み解きたい | [reading_guide.md](reading_guide.md) |
| アルゴリズムの理論を知りたい | [algorithms/](algorithms/) |
| C++ から使いたい | [usage/quickstart_cpp.md](usage/quickstart_cpp.md) |
| Python から使いたい | [usage/quickstart_python.md](usage/quickstart_python.md) |
| シミュレータでロボットを走らせたい | [usage/simulator_guide.md](usage/simulator_guide.md) |
| 開発に参加したい | [contributing/testing.md](contributing/testing.md) |

## ロードマップ

実装は次のマイルストーン順に進む。各マイルストーンの詳細はリポジトリ root の計画書および
`contributing/release_process.md` を参照。

- **M0**: テストハーネスとシナリオ fixture（最優先、旧コードに対する正解値を固定）
- **M1**: コアスケルトンと CI
- **M2**: A\* / Lookahead / GoalChecker / path_postprocessing
- **M3**: `texnitis_mbf_plugins` 連携と旧 `texnitis_move_base_like` 削除
- **M4**: Differential drive Pure Pursuit と Height-aware A\*
- **M5**: MPPI 統合（CasADi optional）
- **M6**: ツール群（waypoint_sender / WebUI / nav_state_publisher）の mbf 移行
- **M7**: pybind11 バインディング
- **M8**: Python シミュレータと disturbance / metrics
- **M9**: ドキュメント仕上げと品質ゲート

## ライセンス

MIT License — リポ root の `LICENSE`。
