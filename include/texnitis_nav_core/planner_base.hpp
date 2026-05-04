/// @file
/// @brief PlannerBase: グローバルプランナーの純粋仮想インターフェース。
///
/// `mbf_simple_core::SimplePlanner` と同じ役割を、ROS 依存ゼロで提供
/// する。mbf アダプタ（`texnitis_mbf_planners`）はこの interface を
/// 内側に持ち、`PoseStamped` ↔ `Pose2D`、`OccupancyGrid` ↔
/// `GridMapView` の変換だけを担当する。
///
/// 読む順序: [reading_guide.md](../../docs/reading_guide.md) を参照。
///
/// 実装側の規約:
///
///   1. `planPath` は **同期** に走り、結果を `out_path` に書き込む。
///      呼び出し元のメインループをブロックする想定。
///   2. 長時間 (>50ms) かかる場合は実装が `cancel()` のフラグを
///      検査し、立っていれば `PlanResult::Cancelled` で抜けること。
///   3. `reset()` は次のゴール投入直前に呼ばれる。`cancel()` フラグの
///      クリアもここで行う。
///   4. ロガーは `setLogger()` で注入される。`nullptr` ならログ出力なし。

#pragma once

#include "texnitis_nav_core/grid_map_view.hpp"
#include "texnitis_nav_core/logger.hpp"
#include "texnitis_nav_core/result.hpp"
#include "texnitis_nav_core/types.hpp"

namespace texnitis::nav_core {

/// @brief プランナーの純粋仮想ベースクラス。
///
/// `~PlannerBase()` は virtual。実装は `unique_ptr<PlannerBase>` で
/// 保持しても安全。
class PlannerBase {
   public:
    virtual ~PlannerBase () = default;

    /// @brief 経路を計画する。
    ///
    /// @param map       占有グリッド。`map.data == nullptr` なら
    ///                  `PlanResult::InvalidMap` を返すこと。
    /// @param start     開始 pose（map 座標系、`yaw` は `[-π, π]`）。
    /// @param goal      ゴール pose。
    /// @param out_path  出力。`Success` 時は最低でもスタート + ゴール
    ///                  を含むこと（経路が 1 セルでも `poses.size() >= 1`）。
    /// @return プラン結果。詳細は @ref texnitis::nav_core::PlanResult。
    [[nodiscard]] virtual PlanResult planPath (const GridMapView &map,
                                               const Pose2D      &start,
                                               const Pose2D      &goal,
                                               Path2D            &out_path) = 0;

    /// @brief 進行中の `planPath` 呼び出しを中断する。
    ///
    /// 実装が長時間ループを持たない場合は no-op で良い。
    /// thread-safe である必要がある（atomic flag 推奨）。
    virtual void cancel () noexcept {}

    /// @brief 内部状態をクリアする。次の `planPath` 呼び出し前に
    ///        呼ばれる想定。`cancel()` のフラグもここでクリアする。
    virtual void reset () {}

    /// @brief ロガーを差し替える。`nullptr` を渡すとログ出力なしに
    ///        なる。デフォルト構築直後は `nullptr`。
    virtual void setLogger (LoggerFn logger) { logger_ = std::move (logger); }

   protected:
    /// @brief ロガーが設定されていれば呼ぶ薄い薄いユーティリティ。
    void log (LogLevel level, std::string_view message) const {
        ::texnitis::nav_core::log (logger_, level, message);
    }

    LoggerFn logger_;
};

}  // namespace texnitis::nav_core
