/// @file
/// @brief XY/yaw 到達判定。`stateful` フラグの取扱いを正確に再現。
///
/// 旧 `texnitis_move_base_like::GoalChecker` は `rclcpp::Node` から
/// パラメータを引いていたが、それをコンストラクタ注入に置き換える。
///
/// `stateful=true` の場合、`isXYReached` は **一度 True を返したら
/// 次に `reset()` するまで True を返し続ける**。yaw 整合中に小さな
/// 揺らぎで XY が外れても並進をやり直さない、という振る舞いの保証
/// （シナリオ S-01 / S-17 がこれを検証する）。

#pragma once

#include "texnitis_nav_core/types.hpp"

#include <cmath>

namespace texnitis::nav_core {

struct GoalCheckerParams {
    double xy_tolerance{0.05};   ///< 位置許容 [m]
    double yaw_tolerance{0.10};  ///< ヨー許容 [rad]
    bool   stateful{true};       ///< 一度到達したら剥がさない
};

class GoalChecker {
   public:
    GoalChecker () = default;
    explicit GoalChecker (const GoalCheckerParams &params) : params_ (params) {}

    /// @brief パラメータを差し替える。`reset()` も同時に呼ぶ。
    void setParams (const GoalCheckerParams &params) {
        params_ = params;
        reset ();
    }

    [[nodiscard]] const GoalCheckerParams &params () const noexcept { return params_; }

    /// @brief XY 到達判定。`stateful=true` のとき一度立ったら剥がさない。
    [[nodiscard]] bool isXYReached (const Pose2D &current, const Pose2D &goal) {
        const double distance = std::hypot (goal.x - current.x, goal.y - current.y);
        const bool in_range = (distance <= params_.xy_tolerance);
        if (params_.stateful) {
            if (in_range) {
                xy_reached_ = true;
            }
            return xy_reached_;
        }
        return in_range;
    }

    /// @brief Yaw 到達判定。stateful 効果は無し（角度はぶれにくいため）。
    [[nodiscard]] bool isYawReached (const Pose2D &current, const Pose2D &goal) const noexcept {
        return std::fabs (shortestAngularDiff (current.yaw, goal.yaw)) <= params_.yaw_tolerance;
    }

    /// @brief 両方到達しているか。
    [[nodiscard]] bool isReached (const Pose2D &current, const Pose2D &goal) {
        return isXYReached (current, goal) && isYawReached (current, goal);
    }

    /// @brief Tolerance を一時的に上書きしての判定。mbf アダプタが
    ///        `isGoalReached(d_tol, a_tol)` で呼ばれたとき、その値を
    ///        ここに渡す。`stateful` は影響を受けない。
    [[nodiscard]] bool isReached (const Pose2D &current,
                                  const Pose2D &goal,
                                  double        xy_override,
                                  double        yaw_override) {
        GoalCheckerParams scratch = params_;
        scratch.xy_tolerance = xy_override;
        scratch.yaw_tolerance = yaw_override;
        const double distance = std::hypot (goal.x - current.x, goal.y - current.y);
        const bool xy_in_range = (distance <= scratch.xy_tolerance);
        if (scratch.stateful && xy_in_range) {
            xy_reached_ = true;
        }
        const bool xy_ok  = scratch.stateful ? xy_reached_ : xy_in_range;
        const bool yaw_ok = std::fabs (shortestAngularDiff (current.yaw, goal.yaw)) <= scratch.yaw_tolerance;
        return xy_ok && yaw_ok;
    }

    /// @brief 内部状態をクリア。新しいゴールを受け付けたとき呼ぶ。
    void reset () noexcept { xy_reached_ = false; }

    /// @brief 現在の stateful フラグの値（テスト用）。
    [[nodiscard]] bool xyReachedFlag () const noexcept { return xy_reached_; }

   private:
    GoalCheckerParams params_;
    bool              xy_reached_{false};
};

}  // namespace texnitis::nav_core
