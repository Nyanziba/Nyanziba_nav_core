/// @file
/// @brief ControllerBase: ローカルコントローラの純粋仮想インターフェース。
///
/// `mbf_simple_core::SimpleController` の役割を ROS 依存ゼロで再現
/// する。mbf アダプタはこの interface を内側に持ち、`Pose2D ⇄
/// PoseStamped`、`Twist2D ⇄ TwistStamped` の変換だけを担当する。
///
/// 旧 `texnitis_move_base_like::ControllerBase` は
/// `computeCommand(pose, path, frame, twist&, bool&)` という単一
/// メソッドだったが、mbf 互換のため `setPlan`/`computeCommand`/
/// `isGoalReached` の三段に分解する。
///
/// 実装側の規約:
///
///   1. `setPlan` は受け取った経路を内部キャッシュに保持し、追従の
///      初期化（lookahead index リセット、stateful フラグの整理など）
///      を行う。`reset()` を併せて呼ぶ実装が一般的。
///   2. `computeCommand` は **状態を持って良い**（last_vx_ 等）。1
///      tick 単位で呼ばれる前提。
///   3. `isGoalReached` は `setPlan` 直後に false を返すべき（経路の
///      末尾 pose を current_pose と比較するだけで True になる、を防ぐ）。
///   4. `cancel()` は thread-safe で atomic flag を立てるだけにする。

#pragma once

#include "texnitis_nav_core/logger.hpp"
#include "texnitis_nav_core/result.hpp"
#include "texnitis_nav_core/types.hpp"

namespace texnitis::nav_core {

class ControllerBase {
   public:
    virtual ~ControllerBase () = default;

    /// @brief 追従対象の経路を渡す。複製してキャッシュすること。
    virtual void setPlan (const Path2D &path) = 0;

    /// @brief 1 tick 分の cmd を計算する。
    ///
    /// @param current_pose 現在 pose（map 座標系）。
    /// @param cmd_out      [out] 計算された Twist。差動駆動なら
    ///                     `vy = 0`、メカナム / ホロノミックなら全成分。
    /// @return ステータス enum。`Success` でも `cmd_out` は変動しうる。
    [[nodiscard]] virtual ControllerResult
    computeCommand (const Pose2D &current_pose, Twist2D &cmd_out) = 0;

    /// @brief ゴールに到達しているか。
    ///
    /// @param dist_tolerance  位置許容（m）。0 を渡すとコンストラクタ
    ///                        から渡された値を使う実装でも、毎呼び出し
    ///                        引数を優先する実装でも良い。
    /// @param angle_tolerance ヨー許容（rad）。同上。
    [[nodiscard]] virtual bool isGoalReached (double dist_tolerance,
                                              double angle_tolerance) const = 0;

    /// @brief 進行中の `computeCommand` 呼び出しを中断する。実装は
    ///        thread-safe な atomic flag を立てるだけにする。
    virtual void cancel () noexcept {}

    /// @brief 内部状態クリア。`last_vx_`, `xy_goal_reached_`, `cancel`
    ///        フラグなどを初期化する。
    virtual void reset () {}

    virtual void setLogger (LoggerFn logger) { logger_ = std::move (logger); }

   protected:
    void log (LogLevel level, std::string_view message) const {
        ::texnitis::nav_core::log (logger_, level, message);
    }

    LoggerFn logger_;
};

}  // namespace texnitis::nav_core
