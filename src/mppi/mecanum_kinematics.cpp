// MecanumKinematics: 旧 mecanum_mppi_controller::MecanumKinematics の
// 純 C++ 移植。アルゴリズムは同等。

#include "texnitis_nav_core/mppi/mecanum_kinematics.hpp"

#include <cmath>

namespace texnitis::nav_core::mppi {

MecanumKinematics::MecanumKinematics (double v_max,
                                       double omega_max,
                                       double dt,
                                       double wheel_base) noexcept
    : v_max_ (v_max), omega_max_ (omega_max), dt_ (dt), wheel_base_ (wheel_base) {}

State MecanumKinematics::updateState (const State &state, const Control &control) const noexcept {
    State next = state;

    // Update velocities in robot frame.
    next (3) = state (3) + control (0) * dt_;
    next (4) = state (4) + control (1) * dt_;
    next (5) = state (5) + control (2) * dt_;

    // Apply linear velocity magnitude constraint.
    const double speed = std::sqrt (next (3) * next (3) + next (4) * next (4));
    if (speed > v_max_) {
        const double scale = v_max_ / speed;
        next (3) *= scale;
        next (4) *= scale;
    }
    if (std::abs (next (5)) > omega_max_) {
        next (5) = std::copysign (omega_max_, next (5));
    }

    // Transform velocity from robot frame to world frame.
    const double cos_theta = std::cos (state (2));
    const double sin_theta = std::sin (state (2));
    const double vx_world  = cos_theta * next (3) - sin_theta * next (4);
    const double vy_world  = sin_theta * next (3) + cos_theta * next (4);

    next (0) = state (0) + vx_world * dt_;
    next (1) = state (1) + vy_world * dt_;
    next (2) = state (2) + next (5) * dt_;

    // Normalise theta into [-pi, pi].
    next (2) = std::atan2 (std::sin (next (2)), std::cos (next (2)));

    return next;
}

Eigen::Vector4d MecanumKinematics::getWheelSpeeds (double vx, double vy, double omega) const noexcept {
    Eigen::Vector4d wheels;
    wheels (0) = vx - vy - wheel_base_ * omega;  // FL
    wheels (1) = vx + vy + wheel_base_ * omega;  // FR
    wheels (2) = vx + vy - wheel_base_ * omega;  // RL
    wheels (3) = vx - vy + wheel_base_ * omega;  // RR
    return wheels;
}

}  // namespace texnitis::nav_core::mppi
