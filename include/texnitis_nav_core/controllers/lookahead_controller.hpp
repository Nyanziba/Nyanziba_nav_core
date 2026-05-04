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
    }

   private:
    LookaheadParams params_{};
    GoalChecker     goal_checker_{};
    Path2D          path_;

    std::atomic<bool> cancel_flag_{false};
    /// `isGoalReached` を `const` にするためのキャッシュ（mutable）。
    /// 実装側は computeCommand 中で更新する。
    mutable bool last_goal_reached_{false};
};

}  // namespace texnitis::nav_core
