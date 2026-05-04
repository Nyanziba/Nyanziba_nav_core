/// @file
/// @brief Eigen のみで動くサンプリングベース MPPI コントローラ。
///
/// 構造はクラシックな MPPI:
///   1. ノミナル制御列 `u_nominal_` を保持
///   2. 各サンプルで `u_i = u_nominal + epsilon_i`、`epsilon` は
///      Gaussian(0, sigma)
///   3. 各サンプルを `MecanumKinematics::updateState` で展開
///   4. 経路追従距離 + ヨー誤差 + 制御コストの和を rollout cost に
///   5. softmax 重み `w_i = exp(-(cost_i - min_cost) / lambda)` で
///      正規化し、加重平均で nominal control を更新
///   6. 最初のステップを実機 cmd として返し、残りは shift して次回へ
///
/// CasADi 統合（解析勾配付き）は M9 の追加コミットで導入予定。

#pragma once

#include <atomic>
#include <random>
#include <vector>

#include <Eigen/Dense>

#include "texnitis_nav_core/controller_base.hpp"
#include "texnitis_nav_core/goal_checker.hpp"
#include "texnitis_nav_core/mppi/mecanum_kinematics.hpp"
#include "texnitis_nav_core/mppi/mppi_params.hpp"

namespace texnitis::nav_core::mppi {

class MecanumMppiController final : public ControllerBase {
   public:
    MecanumMppiController () = default;
    explicit MecanumMppiController (const MppiParams &params);

    void setParams (const MppiParams &params);

    [[nodiscard]] const MppiParams &params () const noexcept { return params_; }

    void setPlan (const Path2D &path) override;

    [[nodiscard]] ControllerResult computeCommand (const Pose2D &current_pose,
                                                   Twist2D      &cmd_out) override;

    [[nodiscard]] bool isGoalReached (double dist_tolerance,
                                      double angle_tolerance) const override;

    void cancel () noexcept override { cancel_flag_.store (true, std::memory_order_relaxed); }

    void reset () override;

   private:
    /// 単一ロールアウトのコスト。
    [[nodiscard]] double rolloutCost (const std::vector<State> &trajectory,
                                       const std::vector<Control> &controls,
                                       size_t                       nearest_path_idx) const noexcept;

    /// 経路上で `pose` に最も近い index を返す。
    [[nodiscard]] size_t nearestIndex (double x, double y) const noexcept;

    MppiParams       params_{};
    GoalChecker      goal_checker_{};
    Path2D           path_;
    MecanumKinematics kinematics_{0.5, 1.5, 0.05};

    /// `[horizon][3]` のノミナル制御列（次回呼び出しで使う）。
    std::vector<Control> u_nominal_;

    std::mt19937_64 rng_{42};

    /// 前回の vy/wz を引き継いで MecanumKinematics に渡すための速度状態。
    Eigen::Vector3d body_velocity_{Eigen::Vector3d::Zero ()};

    std::atomic<bool> cancel_flag_{false};
    mutable bool      last_goal_reached_{false};
};

}  // namespace texnitis::nav_core::mppi
