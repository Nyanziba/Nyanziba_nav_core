/// @file
/// @brief DiffDrivePurePursuitController のパラメータ。

#pragma once

#include "texnitis_nav_core/goal_checker.hpp"

namespace texnitis::nav_core {

struct DiffDrivePurePursuitParams {
    double max_linear_velocity{0.50};      ///< [m/s]
    double min_linear_velocity{0.05};      ///< [m/s]
    double max_angular_velocity{1.00};     ///< [rad/s]
    double max_acceleration{0.50};         ///< [m/s^2]
    double max_angular_acceleration{1.50}; ///< [rad/s^2]

    /// 動的 lookahead 距離 = clamp(speed * lookahead_time, [min, max])。
    double lookahead_time{0.80};
    double min_lookahead_distance{0.30};
    double max_lookahead_distance{1.00};

    /// 旋回中の並進ゲイン: vx = max_linear * (1 - angle_speed_p * |yaw_err|/π)。
    double angle_speed_p{0.80};

    /// 制御周期 [s]。`max_acceleration` を時間積分するために使う。
    double control_dt{0.05};

    /// GoalChecker の挙動（stateful XY フラグなど）。
    GoalCheckerParams goal_checker{};
};

}  // namespace texnitis::nav_core
