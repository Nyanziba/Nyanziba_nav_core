/// @file
/// @brief メカナム駆動の運動学・離散時間積分。Eigen のみ依存。
///
/// 旧 `mecanum_mppi_controller::MecanumKinematics` の純 C++ 移植。
/// 状態は `[x, y, theta, vx, vy, omega]`、制御は `[ax, ay, alpha]`。
/// `updateState` は速度制約と yaw 正規化を含む 1 ステップ離散積分。

#pragma once

#include <Eigen/Dense>

namespace texnitis::nav_core::mppi {

/// 6D 状態ベクトル: [x, y, theta, vx, vy, omega]
///   x, y: 世界座標系での位置 [m]
///   theta: 機体角 [rad]
///   vx, vy: 機体座標系での並進速度 [m/s]
///   omega: 角速度 [rad/s]
using State = Eigen::Matrix<double, 6, 1>;

/// 制御入力: [ax, ay, alpha]
using Control = Eigen::Vector3d;

class MecanumKinematics {
   public:
    /// @param v_max     並進速度の上限 [m/s]
    /// @param omega_max ヨー角速度の上限 [rad/s]
    /// @param dt        離散時間ステップ [s]
    /// @param wheel_base 中心からホイールまでの距離 [m]
    MecanumKinematics (double v_max,
                       double omega_max,
                       double dt,
                       double wheel_base = 0.30) noexcept;

    /// 1 ステップの状態更新。速度制約とヨー正規化を含む。
    [[nodiscard]] State updateState (const State &state, const Control &control) const noexcept;

    /// (vx, vy, omega) からホイール速度 4 輪 [FL, FR, RL, RR] を返す。
    [[nodiscard]] Eigen::Vector4d getWheelSpeeds (double vx, double vy, double omega) const noexcept;

    [[nodiscard]] double maxVelocity () const noexcept { return v_max_; }
    [[nodiscard]] double maxAngularVelocity () const noexcept { return omega_max_; }
    [[nodiscard]] double dt () const noexcept { return dt_; }
    [[nodiscard]] double wheelBase () const noexcept { return wheel_base_; }

   private:
    double v_max_;
    double omega_max_;
    double dt_;
    double wheel_base_;
};

}  // namespace texnitis::nav_core::mppi
