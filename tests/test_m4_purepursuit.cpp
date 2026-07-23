// M4 smoke tests: DiffDrivePurePursuitController.

#include "texnitis_nav_core/controllers/diff_drive_pure_pursuit_controller.hpp"
#include "texnitis_nav_core/types.hpp"

#include <gtest/gtest.h>

namespace tnc = texnitis::nav_core;

TEST (DiffDrivePurePursuit, EmptyPathIsEmpty) {
    tnc::DiffDrivePurePursuitController controller;
    tnc::Twist2D cmd;
    EXPECT_EQ (controller.computeCommand (tnc::Pose2D{}, cmd), tnc::ControllerResult::EmptyPath);
}

TEST (DiffDrivePurePursuit, AccelerationIsLimited) {
    tnc::DiffDrivePurePursuitParams params;
    params.max_acceleration       = 0.50;
    params.control_dt             = 0.10;
    params.min_linear_velocity    = 0.0;
    params.max_linear_velocity    = 0.50;
    params.min_lookahead_distance = 0.20;
    params.max_lookahead_distance = 1.00;
    tnc::DiffDrivePurePursuitController controller (params);

    tnc::Path2D path;
    for (int i = 0; i < 20; ++i) path.poses.push_back ({0.1 * i, 0.0, 0.0});
    controller.setPlan (path);

    tnc::Twist2D cmd;
    const auto first = controller.computeCommand (tnc::Pose2D{0.0, 0.0, 0.0}, cmd);
    EXPECT_EQ (first, tnc::ControllerResult::Success);
    EXPECT_LE (cmd.vx, params.max_acceleration * params.control_dt + 1e-6);
    EXPECT_GE (cmd.vx, 0.0);
    EXPECT_DOUBLE_EQ (cmd.vy, 0.0);
}

TEST (DiffDrivePurePursuit, ReachesGoalAndReturnsGoalReached) {
    tnc::DiffDrivePurePursuitParams params;
    params.goal_checker.xy_tolerance  = 0.10;
    params.goal_checker.yaw_tolerance = 0.20;
    params.goal_checker.stateful      = true;
    tnc::DiffDrivePurePursuitController controller (params);
    tnc::Path2D path;
    path.poses = {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    controller.setPlan (path);

    tnc::Twist2D cmd;
    const auto status = controller.computeCommand (tnc::Pose2D{1.0, 0.0, 0.0}, cmd);
    EXPECT_EQ (status, tnc::ControllerResult::GoalReached);
    EXPECT_DOUBLE_EQ (cmd.vx, 0.0);
    EXPECT_TRUE (controller.isGoalReached (params.goal_checker.xy_tolerance, params.goal_checker.yaw_tolerance));
}
