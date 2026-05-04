/// @file
/// @brief LookaheadController のパラメータ。POD で構造体定義のみ。

#pragma once

#include "texnitis_nav_core/goal_checker.hpp"

namespace texnitis::nav_core {

struct LookaheadParams {
    /// 並進ゲイン。
    double kp_xy{1.0};

    /// ヨーゲイン。
    double kp_yaw{1.2};

    /// 並進速度上限 [m/s]。
    double max_speed_xy{0.25};

    /// ヨー角速度上限 [rad/s]。
    double max_speed_yaw{0.8};

    /// Lookahead 距離 [m]。
    double lookahead_dist{0.40};

    /// 並進指令の閾値 [m/s]。これ以上 cmd_vel が出ているときに
    /// `max_wz_when_moving` を適用してヨーをクランプする。
    double linear_threshold_for_wz{0.01};

    /// 並進中に許容する最大ヨー角速度 [rad/s]。0 で「動いている間
    /// は回転しない」モード。
    double max_wz_when_moving{0.0};

    /// true なら差動駆動として `vy=0` を強制し、yaw 整合を先に取る。
    bool use_diff_drive{false};

    /// GoalChecker の挙動。`stateful=true` で 1 度到達したら剥がさない。
    GoalCheckerParams goal_checker{};
};

}  // namespace texnitis::nav_core
