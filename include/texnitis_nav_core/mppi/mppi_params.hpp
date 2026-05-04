/// @file
/// @brief MPPI コントローラのパラメータ。

#pragma once

#include <Eigen/Dense>

#include "texnitis_nav_core/goal_checker.hpp"

namespace texnitis::nav_core::mppi {

struct MppiParams {
    /// 予測ホライズン（discrete steps）。
    int horizon{25};

    /// 1 ステップあたりのサンプル数。多いほど精度上がるが計算重い。
    int num_samples{256};

    /// MPPI の温度（重み付け softmax のパラメータ）。
    double lambda{0.10};

    /// 制御サンプリング標準偏差 [ax_sigma, ay_sigma, alpha_sigma]。
    Eigen::Vector3d sigma{Eigen::Vector3d (0.30, 0.30, 0.40)};

    /// 制御絶対値の上限 [ax_max, ay_max, alpha_max]。
    Eigen::Vector3d u_max{Eigen::Vector3d (1.50, 1.50, 1.50)};

    /// 離散時間ステップ [s]。Kinematics に渡される。
    double dt{0.05};

    /// 経路追従コストの重み。
    double w_track{1.0};

    /// 制御コストの重み（過大な制御を抑制）。
    double w_control{0.05};

    /// ヨー追従コストの重み。
    double w_yaw{0.30};

    /// 機体パラメータ。
    double v_max{0.50};
    double omega_max{1.50};
    double wheel_base{0.30};

    /// 乱数 seed（再現性）。0 のとき固定 seed を使う。
    uint64_t seed{42u};

    /// GoalChecker。
    GoalCheckerParams goal_checker{};
};

}  // namespace texnitis::nav_core::mppi
