#include "texnitis_nav_core/motion_models/holonomic_motion_model.hpp"

#include <algorithm>
#include <cmath>

namespace texnitis::nav_core {
namespace {
MotionPrimitive linearPrimitive (double x, double y, double yaw) {
    MotionPrimitive p;
    p.length        = std::hypot (x, y);
    p.delta_yaw     = yaw;
    constexpr int n = 6;
    for (int i = 1; i <= n; ++i) {
        const double t = static_cast<double> (i) / n;
        p.samples.push_back ({x * t, y * t, yaw * t});
    }
    p.in_place = p.length < 1e-9;
    return p;
}
}  // namespace

HolonomicMotionModel::HolonomicMotionModel (HolonomicMotionModelParams params) : params_ (params) {
    const double l = params_.primitive_length;
    for (int ix = -1; ix <= 1; ++ix)
        for (int iy = -1; iy <= 1; ++iy) {
            if (ix == 0 && iy == 0) continue;
            const double scale = (ix != 0 && iy != 0) ? 1.0 / std::sqrt (2.0) : 1.0;
            primitives_.push_back (linearPrimitive (ix * l * scale, iy * l * scale, 0.0));
        }
    const double dyaw = 2.0 * kPi / std::max (4, params_.heading_bins);
    primitives_.push_back (linearPrimitive (0.0, 0.0, dyaw));
    primitives_.push_back (linearPrimitive (0.0, 0.0, -dyaw));
    if (params_.allow_translation_while_rotating) {
        primitives_.push_back (linearPrimitive (l, 0.0, dyaw));
        primitives_.push_back (linearPrimitive (l, 0.0, -dyaw));
    }
}

bool HolonomicMotionModel::valid () const noexcept {
    return params_.primitive_length > 0.0 && params_.heading_bins >= 4 && params_.max_velocity_x > 0.0 && params_.max_velocity_y > 0.0 && params_.max_acceleration_x > 0.0 && params_.max_acceleration_y > 0.0 && params_.max_angular_velocity > 0.0 &&
           params_.max_angular_acceleration > 0.0;
}

double HolonomicMotionModel::transitionTime (const MotionPrimitive &p) const noexcept {
    const Pose2D end = p.samples.empty () ? Pose2D{} : p.samples.back ();
    const double tx  = trapezoidalTravelTime (end.x, params_.max_velocity_x, params_.max_acceleration_x);
    const double ty  = trapezoidalTravelTime (end.y, params_.max_velocity_y, params_.max_acceleration_y);
    const double tr  = trapezoidalTravelTime (p.delta_yaw, params_.max_angular_velocity, params_.max_angular_acceleration);
    return params_.allow_translation_while_rotating ? std::max ({tx, ty, tr}) : std::max (tx, ty) + tr;
}

double HolonomicMotionModel::lowerBoundTime (const Pose2D &from, const Pose2D &goal) const noexcept {
    const double linear_v = std::max (params_.max_velocity_x, params_.max_velocity_y);
    const double txy      = distanceXY (from, goal) / linear_v;
    const double tyaw     = std::abs (shortestAngularDiff (from.yaw, goal.yaw)) / params_.max_angular_velocity;
    return params_.allow_translation_while_rotating ? std::max (txy, tyaw) : txy + tyaw;
}

}  // namespace texnitis::nav_core
