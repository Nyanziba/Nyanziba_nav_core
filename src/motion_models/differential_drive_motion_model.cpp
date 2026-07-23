#include "texnitis_nav_core/motion_models/differential_drive_motion_model.hpp"

#include <algorithm>
#include <cmath>

namespace texnitis::nav_core {
namespace {
MotionPrimitive arcPrimitive (double length, double yaw, bool reverse = false) {
    MotionPrimitive p;
    p.length        = std::abs (length);
    p.delta_yaw     = yaw;
    p.reverse       = reverse;
    constexpr int n = 8;
    for (int i = 1; i <= n; ++i) {
        const double t = static_cast<double> (i) / n;
        if (std::abs (yaw) < 1e-9)
            p.samples.push_back ({length * t, 0.0, 0.0});
        else {
            const double radius = length / yaw;
            const double a      = yaw * t;
            p.samples.push_back ({radius * std::sin (a), radius * (1.0 - std::cos (a)), a});
        }
    }
    return p;
}
MotionPrimitive rotationPrimitive (double yaw) {
    MotionPrimitive p = arcPrimitive (0.0, yaw);
    p.in_place        = true;
    return p;
}
}  // namespace

DifferentialDriveMotionModel::DifferentialDriveMotionModel (DifferentialDriveMotionModelParams params) : params_ (params) {
    const double l    = params_.primitive_length;
    const double dyaw = std::min (2.0 * kPi / std::max (4, params_.heading_bins), l / std::max (1e-6, params_.minimum_turning_radius));
    primitives_.push_back (arcPrimitive (l, 0.0));
    primitives_.push_back (arcPrimitive (l, dyaw));
    primitives_.push_back (arcPrimitive (l, -dyaw));
    if (params_.allow_in_place_rotation) {
        const double bin = 2.0 * kPi / std::max (4, params_.heading_bins);
        primitives_.push_back (rotationPrimitive (bin));
        primitives_.push_back (rotationPrimitive (-bin));
    }
    if (params_.allow_reverse) {
        primitives_.push_back (arcPrimitive (-l, 0.0, true));
        primitives_.push_back (arcPrimitive (-l, -dyaw, true));
        primitives_.push_back (arcPrimitive (-l, dyaw, true));
    }
}

bool DifferentialDriveMotionModel::valid () const noexcept {
    return params_.primitive_length > 0.0 && params_.heading_bins >= 4 && params_.max_linear_velocity > 0.0 && params_.max_linear_acceleration > 0.0 && params_.max_angular_velocity > 0.0 && params_.max_angular_acceleration > 0.0 &&
           params_.minimum_turning_radius > 0.0 && params_.reverse_penalty >= 0.0;
}

double DifferentialDriveMotionModel::transitionTime (const MotionPrimitive &p) const noexcept {
    const double tl = trapezoidalTravelTime (p.length, params_.max_linear_velocity, params_.max_linear_acceleration);
    const double ta = trapezoidalTravelTime (p.delta_yaw, params_.max_angular_velocity, params_.max_angular_acceleration);
    return std::max (tl, ta) + (p.reverse ? params_.reverse_penalty : 0.0);
}

double DifferentialDriveMotionModel::lowerBoundTime (const Pose2D &from, const Pose2D &goal) const noexcept {
    return std::max (distanceXY (from, goal) / params_.max_linear_velocity, std::abs (shortestAngularDiff (from.yaw, goal.yaw)) / params_.max_angular_velocity);
}

}  // namespace texnitis::nav_core
