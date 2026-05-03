# 旧 `texnitis_move_base_like` からの移植記録

> このファイルは骨格段階。実装が進むにつれて「どのコミットで何を移植したか」「型変換の対応表」「捨てた機能と理由」を追記する。

## 移植元コミット

- リポジトリ: `nyanziba/.../R2_packages/navigation/texnitis_move_base_like`
- 削除直前のタグ予定: `pre-mbf-migration`

## 名前空間の対応

| 旧 | 新 |
|---|---|
| `texnitis_move_base_like` | `texnitis::nav_core` |
| `mecanum_mppi_controller` | `texnitis::nav_core::mppi` |

## 型の対応

| 旧 | 新 |
|---|---|
| `nav_msgs::msg::OccupancyGrid` | `texnitis::nav_core::GridMapView`（zero-copy ビュー） |
| `nav_msgs::msg::Path` | `texnitis::nav_core::Path2D` |
| `geometry_msgs::msg::Twist` | `texnitis::nav_core::Twist2D` |
| `geometry_msgs::msg::PoseStamped` | `texnitis::nav_core::Pose2D`（frame_id は呼び出し側で管理） |
| `nav_msgs::msg::Odometry` | `(Pose2D, Twist2D)` の二引数に分解 |

## 削除する機能

- 旧 `NavigateToPose.action`、`SetGoal.srv`、`CancelGoal.srv`、`SetPath.srv` は `mbf_msgs/action/{MoveBase,GetPath,ExePath}` に置換
- `move_base_like_node` 自体（mbf プラグインに置き換え）

## 互換維持しないもの

- 並行運用は行わない。M3 完了時に `pre-mbf-migration` タグを打って旧パッケージは削除
