# texnitis_nav_core ドキュメント

ROS から独立した C++ ナビゲーションコア。A\* / Pure Pursuit / MPPI を
C++ から、Python から、`move_base_flex` プラグインから同じ実装で呼べる
ようにすることを目的とする。

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
| パラメータの全リファレンス | [usage/parameters.md](usage/parameters.md) |
| 開発に参加したい | [contributing/testing.md](contributing/testing.md) |

## 進捗

実装は次のマイルストーン順に進めた。詳細は各 commit / git tag を参照。

| 段階 | 内容 |
|---|---|
| **M0** | テスト fixture と legacy_harness で旧コード等価性を固定 |
| **M1** | 共通型・GridMapView・LoggerFn・Result enum |
| **M2** | A\* / Lookahead / GoalChecker をコアへ移植 |
| **M3** | mbf アダプタ（AStar / Lookahead）+ bringup |
| **M4** | HeightAwareAStar / DiffDrivePurePursuit + mbf アダプタ |
| **M5** | MecanumKinematics + Eigen-only MPPI コントローラ |
| **M6** | tools (waypoint_sender / nav_state_publisher) と WebUI |
| **M7** | pybind11 バインディング |
| **M8** | Python 2D シミュレータと sim シナリオ pytest |
| **M9** | アルゴリズム解説・設計判断・利用ドキュメント仕上げ |

## ライセンス

MIT License — リポ root の `LICENSE`。
