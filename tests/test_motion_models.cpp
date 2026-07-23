#include "texnitis_nav_core/motion_models/differential_drive_motion_model.hpp"
#include "texnitis_nav_core/motion_models/holonomic_motion_model.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace nc = texnitis::nav_core;

TEST (MotionModels, HolonomicProvidesLateralPrimitive) {
    nc::HolonomicMotionModel model;
    bool                     lateral_found = false;
    for (const auto &p : model.primitives ()) {
        if (!p.samples.empty () && std::abs (p.samples.back ().y) > 1e-6 && std::abs (p.samples.back ().x) < 1e-6) lateral_found = true;
    }
    EXPECT_TRUE (lateral_found);
}

TEST (MotionModels, DifferentialPrimitivesDoNotStrafe) {
    nc::DifferentialDriveMotionModel model;
    for (const auto &p : model.primitives ()) {
        if (p.samples.empty () || p.in_place) continue;
        const auto &end = p.samples.back ();
        if (std::abs (p.delta_yaw) < 1e-9)
            EXPECT_NEAR (end.y, 0.0, 1e-9);
        else
            EXPECT_NEAR (std::hypot (end.x, end.y) / std::abs (2.0 * std::sin (p.delta_yaw / 2.0)), 0.30 / std::abs (p.delta_yaw), 1e-6);
    }
}

TEST (MotionModels, ReversePrimitiveOnlyExistsWhenEnabled) {
    nc::DifferentialDriveMotionModel forward_only;
    for (const auto &p : forward_only.primitives ()) EXPECT_FALSE (p.reverse);

    nc::DifferentialDriveMotionModelParams params;
    params.allow_reverse = true;
    nc::DifferentialDriveMotionModel with_reverse (params);
    bool                             found = false;
    for (const auto &p : with_reverse.primitives ()) found = found || p.reverse;
    EXPECT_TRUE (found);
}

TEST (MotionModels, FasterLimitsReduceTransitionTime) {
    nc::HolonomicMotionModelParams slow_params;
    slow_params.max_velocity_x = 0.5;
    slow_params.max_velocity_y = 0.5;
    nc::HolonomicMotionModel       slow (slow_params);
    nc::HolonomicMotionModelParams fast_params;
    fast_params.max_velocity_x     = 2.0;
    fast_params.max_velocity_y     = 2.0;
    fast_params.max_acceleration_x = 4.0;
    fast_params.max_acceleration_y = 4.0;
    nc::HolonomicMotionModel fast (fast_params);
    EXPECT_LT (fast.transitionTime (fast.primitives ().front ()), slow.transitionTime (slow.primitives ().front ()));
}

TEST (MotionModels, RejectsZeroVelocityLimit) {
    nc::HolonomicMotionModelParams params;
    params.max_velocity_x = 0.0;
    EXPECT_FALSE (nc::HolonomicMotionModel (params).valid ());
}
