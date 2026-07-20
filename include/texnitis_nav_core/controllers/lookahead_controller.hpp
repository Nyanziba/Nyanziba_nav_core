/// @file
/// @brief Lookahead 追従コントローラ。差動 / ホロノミック切替可。
///
/// 旧 `texnitis_move_base_like::LookaheadController` の純 C++ 移植。
/// `nav_msgs::msg::Path` ではなく `Path2D` を扱い、`geometry_msgs::
/// msg::Twist` ではなく `Twist2D` を返す。
///
/// @see docs/algorithms/lookahead_controller.md

#pragma once

#include "texnitis_nav_core/controller_base.hpp"
#include "texnitis_nav_core/controllers/lookahead_params.hpp"
#include "texnitis_nav_core/goal_checker.hpp"

#include <atomic>
#include <optional>
#include <vector>

namespace texnitis::nav_core {

class LookaheadController final : public ControllerBase {
   public:
    LookaheadController () = default;
    explicit LookaheadController (const LookaheadParams &params)
        : params_ (params), goal_checker_ (params.goal_checker) {}

    void setParams (const LookaheadParams &params) {
        params_ = params;
        goal_checker_.setParams (params.goal_checker);
    }

    [[nodiscard]] const LookaheadParams &params () const noexcept { return params_; }

    void setPlan (const Path2D &path) override;

    [[nodiscard]] ControllerResult computeCommand (const Pose2D &current_pose,
                                                   Twist2D      &cmd_out) override;

    [[nodiscard]] bool isGoalReached (double dist_tolerance,
                                      double angle_tolerance) const override;

    void cancel () noexcept override { cancel_flag_.store (true, std::memory_order_relaxed); }

    void reset () override {
        cancel_flag_.store (false, std::memory_order_relaxed);
        goal_checker_.reset ();
        path_.poses.clear ();
        cumulative_arc_lengths_.clear ();
        start_yaw_.reset ();
    }

   private:
    /// rotate_while_moving 用: 最近傍 index までの走破率 [0,1] を返す。
    [[nodiscard]] double progressAt (size_t nearest_idx) const;

    /// rotate_while_moving 用: 進捗率から補間した目標 yaw を返す。
    [[nodiscard]] double interpolatedTargetYaw (double progress) const;

    LookaheadParams params_{};
    GoalChecker     goal_checker_{};
    Path2D          path_;

    /// setPlan 時に前計算する経路の累積弧長。poses と同じ要素数。
    std::vector<double> cumulative_arc_lengths_;

    /// rotate_while_moving の補間開始 yaw。setPlan 後の最初の
    /// computeCommand で現在姿勢から捕捉する。
    std::optional<double> start_yaw_;

    std::atomic<bool> cancel_flag_{false};
    /// `isGoalReached` を `const` にするためのキャッシュ（mutable）。
    /// 実装側は computeCommand 中で更新する。
    mutable bool last_goal_reached_{false};
};

}  // namespace texnitis::nav_core
