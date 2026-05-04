/// @file
/// @brief 差動駆動 Pure Pursuit コントローラ。
///
/// 旧 `texnitis_move_base_like::DiffDrivePurePursuitController` の純
/// C++ 移植。speed-scaled lookahead、加速度上限、stateful GoalChecker
/// を持つ。曲率ベース減速の細部は M9 で詰める想定だが、加速度制限と
/// 接線方向追従は M4 で揃える。

#pragma once

#include "texnitis_nav_core/controller_base.hpp"
#include "texnitis_nav_core/controllers/diff_drive_pure_pursuit_params.hpp"
#include "texnitis_nav_core/goal_checker.hpp"

#include <atomic>
#include <cstddef>

namespace texnitis::nav_core {

class DiffDrivePurePursuitController final : public ControllerBase {
   public:
    DiffDrivePurePursuitController () = default;
    explicit DiffDrivePurePursuitController (const DiffDrivePurePursuitParams &params)
        : params_ (params), goal_checker_ (params.goal_checker) {}

    void setParams (const DiffDrivePurePursuitParams &params) {
        params_ = params;
        goal_checker_.setParams (params.goal_checker);
    }

    [[nodiscard]] const DiffDrivePurePursuitParams &params () const noexcept { return params_; }

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
        last_vx_ = 0.0;
        last_wz_ = 0.0;
        last_goal_reached_ = false;
    }

   private:
    [[nodiscard]] size_t findClosestIndex (const Pose2D &current) const;
    [[nodiscard]] size_t findLookaheadIndex (size_t closest_idx, double lookahead_dist) const;

    DiffDrivePurePursuitParams params_{};
    GoalChecker                goal_checker_{};
    Path2D                     path_;

    double last_vx_{0.0};
    double last_wz_{0.0};

    std::atomic<bool> cancel_flag_{false};
    mutable bool      last_goal_reached_{false};
};

}  // namespace texnitis::nav_core
