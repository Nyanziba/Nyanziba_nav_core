// M2 smoke tests: A* planner, Lookahead controller, GoalChecker.
// Heavier scenario-driven equivalence with the legacy fixtures runs in
// the legacy_harness compare step; here we sanity-check the public API
// shape and that obvious paths / single-step controller calls work.

#include "texnitis_nav_core/controllers/lookahead_controller.hpp"
#include "texnitis_nav_core/goal_checker.hpp"
#include "texnitis_nav_core/grid_map_view.hpp"
#include "texnitis_nav_core/planners/astar_planner.hpp"
#include "texnitis_nav_core/types.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace tnc = texnitis::nav_core;

namespace {

tnc::GridMapView makeOpenGrid (std::vector<int8_t> &storage,
                               int width, int height,
                               double resolution = 0.1,
                               double origin_x   = 0.0,
                               double origin_y   = 0.0) {
    storage.assign (static_cast<size_t> (width) * height, 0);
    tnc::GridMapView m;
    m.data       = storage.data ();
    m.width      = width;
    m.height     = height;
    m.resolution = resolution;
    m.origin_x   = origin_x;
    m.origin_y   = origin_y;
    return m;
}

}  // namespace

TEST (AStarPlanner, OpenGridProducesPath) {
    std::vector<int8_t> grid;
    tnc::GridMapView m = makeOpenGrid (grid, 10, 10);

    tnc::AStarParams params;
    params.inflation_radius   = 0.0;
    params.unknown_is_obstacle = false;
    tnc::AStarPlanner planner (params);

    tnc::Path2D path;
    const auto status = planner.planPath (m, tnc::Pose2D{0.05, 0.05, 0.0},
                                          tnc::Pose2D{0.85, 0.85, 0.0}, path);
    EXPECT_EQ (status, tnc::PlanResult::Success);
    EXPECT_GE (path.poses.size (), 2u);
    EXPECT_NEAR (path.poses.back ().x, 0.85, 0.1);
    EXPECT_NEAR (path.poses.back ().y, 0.85, 0.1);
    EXPECT_NEAR (path.poses.back ().yaw, 0.0, 1e-9);
}

TEST (AStarPlanner, NullDataIsInvalidMap) {
    tnc::GridMapView m;
    tnc::AStarPlanner planner;
    tnc::Path2D path;
    EXPECT_EQ (planner.planPath (m, {}, {}, path), tnc::PlanResult::InvalidMap);
}

TEST (AStarPlanner, GoalInWallReturnsCollision) {
    std::vector<int8_t> grid (100, 100);  // entire map blocked
    tnc::GridMapView m = makeOpenGrid (grid, 10, 10);
    for (auto &v : grid) v = 100;

    tnc::AStarParams params;
    params.inflation_radius   = 0.0;
    params.unknown_is_obstacle = true;
    tnc::AStarPlanner planner (params);

    tnc::Path2D path;
    const auto status = planner.planPath (m, tnc::Pose2D{0.05, 0.05, 0.0},
                                          tnc::Pose2D{0.85, 0.85, 0.0}, path);
    EXPECT_TRUE (status == tnc::PlanResult::StartInCollision ||
                 status == tnc::PlanResult::GoalInCollision ||
                 status == tnc::PlanResult::NoPathFound);
    EXPECT_TRUE (path.poses.empty ());
}

TEST (AStarPlanner, IsDeterministicForRepeatedCalls) {
    std::vector<int8_t> grid;
    tnc::GridMapView m = makeOpenGrid (grid, 10, 10);
    tnc::AStarPlanner planner;

    tnc::Path2D first;
    tnc::Path2D second;
    EXPECT_EQ (planner.planPath (m, tnc::Pose2D{0.05, 0.05, 0.0},
                                 tnc::Pose2D{0.85, 0.45, 0.0}, first),
               tnc::PlanResult::Success);
    EXPECT_EQ (planner.planPath (m, tnc::Pose2D{0.05, 0.05, 0.0},
                                 tnc::Pose2D{0.85, 0.45, 0.0}, second),
               tnc::PlanResult::Success);
    ASSERT_EQ (first.poses.size (), second.poses.size ());
    for (size_t i = 0; i < first.poses.size (); ++i) {
        EXPECT_DOUBLE_EQ (first.poses[i].x, second.poses[i].x);
        EXPECT_DOUBLE_EQ (first.poses[i].y, second.poses[i].y);
    }
}

TEST (LookaheadController, EmptyPathReportsEmptyPath) {
    tnc::LookaheadController controller;
    tnc::Twist2D cmd;
    EXPECT_EQ (controller.computeCommand (tnc::Pose2D{}, cmd),
               tnc::ControllerResult::EmptyPath);
    EXPECT_DOUBLE_EQ (cmd.vx, 0.0);
    EXPECT_DOUBLE_EQ (cmd.vy, 0.0);
    EXPECT_DOUBLE_EQ (cmd.wz, 0.0);
}

TEST (LookaheadController, GoalReachedProducesZeroCommand) {
    tnc::LookaheadParams params;
    params.use_diff_drive = false;
    tnc::LookaheadController controller (params);

    tnc::Path2D path;
    path.poses.push_back ({0.0, 0.0, 0.0});
    path.poses.push_back ({0.5, 0.0, 0.0});
    path.poses.push_back ({1.0, 0.0, 0.0});
    controller.setPlan (path);

    tnc::Twist2D cmd;
    const auto result = controller.computeCommand (tnc::Pose2D{1.0, 0.0, 0.0}, cmd);
    EXPECT_EQ (result, tnc::ControllerResult::GoalReached);
    EXPECT_DOUBLE_EQ (cmd.vx, 0.0);
    EXPECT_DOUBLE_EQ (cmd.wz, 0.0);
}

TEST (LookaheadController, MovesTowardLookaheadPoint) {
    tnc::LookaheadParams params;
    params.use_diff_drive = false;
    params.kp_xy          = 1.0;
    params.max_speed_xy   = 0.5;
    params.lookahead_dist = 0.4;
    tnc::LookaheadController controller (params);

    tnc::Path2D path;
    for (int i = 0; i < 10; ++i) {
        path.poses.push_back ({0.1 * i, 0.0, 0.0});
    }
    controller.setPlan (path);

    tnc::Twist2D cmd;
    const auto status = controller.computeCommand (tnc::Pose2D{0.0, 0.0, 0.0}, cmd);
    EXPECT_EQ (status, tnc::ControllerResult::Success);
    EXPECT_GT (cmd.vx, 0.0);
    EXPECT_LE (cmd.vx, params.max_speed_xy);
}

namespace {

// 原点から +x に 1.0 m 直進し、ゴールで yaw = pi/2 を要求する経路。
tnc::Path2D makeStraightPathWithGoalYaw (double goal_yaw) {
    tnc::Path2D path;
    for (int i = 0; i <= 10; ++i) {
        path.poses.push_back ({0.1 * i, 0.0, 0.0});
    }
    path.poses.back ().yaw = goal_yaw;
    return path;
}

tnc::LookaheadParams makeRotateWhileMovingParams () {
    tnc::LookaheadParams params;
    params.use_diff_drive               = false;
    params.kp_xy                        = 1.0;
    params.kp_yaw                       = 1.0;
    params.max_speed_xy                 = 0.5;
    params.max_speed_yaw                = 10.0;  // クランプさせず生の指令を観測する
    params.lookahead_dist               = 0.2;
    params.rotate_while_moving          = true;
    params.rotate_while_moving_exponent = 2.0;
    return params;
}

}  // namespace

TEST (LookaheadController, RotateWhileMovingRampsUpYawNearGoal) {
    constexpr double kGoalYaw = 1.5707963267948966;  // pi/2
    tnc::LookaheadController controller (makeRotateWhileMovingParams ());
    controller.setPlan (makeStraightPathWithGoalYaw (kGoalYaw));

    // 経路開始点: 進捗 0 なので回転指令はほぼゼロ。
    tnc::Twist2D cmd_at_start;
    ASSERT_EQ (controller.computeCommand (tnc::Pose2D{0.0, 0.0, 0.0}, cmd_at_start),
               tnc::ControllerResult::Success);
    EXPECT_NEAR (cmd_at_start.wz, 0.0, 1e-6);
    EXPECT_GT (cmd_at_start.vx, 0.0);

    // 中間点 (進捗 0.5): 補間目標は goal_yaw * 0.5^2 = goal_yaw / 4。
    tnc::Twist2D cmd_at_middle;
    ASSERT_EQ (controller.computeCommand (tnc::Pose2D{0.5, 0.0, 0.0}, cmd_at_middle),
               tnc::ControllerResult::Success);
    EXPECT_NEAR (cmd_at_middle.wz, kGoalYaw * 0.25, 1e-6);

    // ゴール直前 (進捗 0.9): 補間目標は goal_yaw * 0.81。中間より急に回る。
    tnc::Twist2D cmd_near_goal;
    ASSERT_EQ (controller.computeCommand (tnc::Pose2D{0.9, 0.0, 0.0}, cmd_near_goal),
               tnc::ControllerResult::Success);
    EXPECT_NEAR (cmd_near_goal.wz, kGoalYaw * 0.81, 1e-6);
    EXPECT_GT (cmd_near_goal.wz, cmd_at_middle.wz);
}

TEST (LookaheadController, RotateWhileMovingIgnoresMaxWzWhenMovingClamp) {
    constexpr double kGoalYaw = 1.5707963267948966;
    auto params = makeRotateWhileMovingParams ();
    params.max_wz_when_moving      = 0.0;  // 従来モードなら回転ゼロに潰される設定
    params.linear_threshold_for_wz = 0.0;
    tnc::LookaheadController controller (params);
    controller.setPlan (makeStraightPathWithGoalYaw (kGoalYaw));

    tnc::Twist2D cmd;
    ASSERT_EQ (controller.computeCommand (tnc::Pose2D{0.5, 0.0, 0.0}, cmd),
               tnc::ControllerResult::Success);
    EXPECT_GT (cmd.wz, 0.0);
}

TEST (LookaheadController, DefaultKeepsHeadingTowardTravelDirection) {
    constexpr double kGoalYaw = 1.5707963267948966;
    auto params = makeRotateWhileMovingParams ();
    params.rotate_while_moving = false;
    tnc::LookaheadController controller (params);
    controller.setPlan (makeStraightPathWithGoalYaw (kGoalYaw));

    // 従来挙動: 移動中の目標 yaw は進行方向 (=0)。ゴール yaw は見ない。
    tnc::Twist2D cmd;
    ASSERT_EQ (controller.computeCommand (tnc::Pose2D{0.5, 0.0, 0.0}, cmd),
               tnc::ControllerResult::Success);
    EXPECT_NEAR (cmd.wz, 0.0, 1e-6);
}

TEST (GoalChecker, StatefulFlagPersistsAfterFirstReach) {
    tnc::GoalCheckerParams params;
    params.xy_tolerance  = 0.1;
    params.yaw_tolerance = 0.1;
    params.stateful      = true;
    tnc::GoalChecker checker (params);

    const tnc::Pose2D goal{0.5, 0.5, 0.0};
    EXPECT_TRUE (checker.isXYReached ({0.5, 0.5, 0.0}, goal));
    EXPECT_TRUE (checker.isXYReached ({0.7, 0.7, 0.0}, goal));  // stays True
    checker.reset ();
    EXPECT_FALSE (checker.isXYReached ({0.7, 0.7, 0.0}, goal));
}

TEST (GoalChecker, NonStatefulFlagFollowsLiveDistance) {
    tnc::GoalCheckerParams params;
    params.xy_tolerance  = 0.1;
    params.yaw_tolerance = 0.1;
    params.stateful      = false;
    tnc::GoalChecker checker (params);

    const tnc::Pose2D goal{0.5, 0.5, 0.0};
    EXPECT_TRUE (checker.isXYReached ({0.5, 0.5, 0.0}, goal));
    EXPECT_FALSE (checker.isXYReached ({0.7, 0.7, 0.0}, goal));
}
