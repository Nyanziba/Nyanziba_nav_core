// M5 smoke tests for the Eigen-only MPPI controller and the mecanum
// kinematics that backs it.

#include "texnitis_nav_core/mppi/mecanum_kinematics.hpp"
#include "texnitis_nav_core/mppi/mecanum_mppi_controller.hpp"
#include "texnitis_nav_core/types.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace tnm = texnitis::nav_core::mppi;
namespace tnc = texnitis::nav_core;

TEST (MecanumKinematics, RestStaysAtRest) {
    tnm::MecanumKinematics k (1.0, 1.0, 0.05);
    tnm::State s = tnm::State::Zero ();
    const auto next = k.updateState (s, tnm::Control::Zero ());
    EXPECT_DOUBLE_EQ (next (0), 0.0);
    EXPECT_DOUBLE_EQ (next (1), 0.0);
    EXPECT_DOUBLE_EQ (next (2), 0.0);
    EXPECT_DOUBLE_EQ (next (3), 0.0);
    EXPECT_DOUBLE_EQ (next (4), 0.0);
    EXPECT_DOUBLE_EQ (next (5), 0.0);
}

TEST (MecanumKinematics, ForwardAccelMovesPositiveX) {
    tnm::MecanumKinematics k (1.0, 1.0, 0.10);
    tnm::State s = tnm::State::Zero ();
    tnm::Control u; u << 1.0, 0.0, 0.0;       // ax = 1 m/s^2
    const auto step1 = k.updateState (s, u);
    EXPECT_GT (step1 (3), 0.0);                // vx > 0 after one step
    EXPECT_GT (step1 (0), 0.0);                // x advanced in world frame
}

TEST (MecanumKinematics, VelocityMagnitudeIsClamped) {
    tnm::MecanumKinematics k (0.5, 1.0, 0.10);
    tnm::State s = tnm::State::Zero ();
    s (3) = 1.0;                               // already past v_max
    const auto next = k.updateState (s, tnm::Control::Zero ());
    EXPECT_LE (std::hypot (next (3), next (4)), 0.5 + 1e-9);
}

TEST (MecanumMppi, EmptyPathIsEmpty) {
    tnm::MppiParams p;
    p.horizon = 5;
    p.num_samples = 16;
    tnm::MecanumMppiController c (p);
    tnc::Twist2D cmd;
    EXPECT_EQ (c.computeCommand (tnc::Pose2D{}, cmd), tnc::ControllerResult::EmptyPath);
}

TEST (MecanumMppi, ProducesNonZeroCommandTowardGoal) {
    tnm::MppiParams p;
    p.horizon              = 10;
    p.num_samples          = 64;
    p.dt                   = 0.10;
    p.lambda               = 0.10;
    p.sigma                = Eigen::Vector3d (0.5, 0.5, 0.5);
    p.u_max                = Eigen::Vector3d (2.0, 2.0, 2.0);
    p.v_max                = 0.5;
    p.omega_max            = 1.5;
    p.goal_checker.xy_tolerance  = 0.05;
    p.goal_checker.yaw_tolerance = 0.20;
    p.goal_checker.stateful      = true;
    tnm::MecanumMppiController c (p);

    tnc::Path2D path;
    for (int i = 0; i <= 10; ++i) {
        path.poses.push_back ({0.10 * i, 0.0, 0.0});
    }
    c.setPlan (path);

    tnc::Twist2D cmd;
    const auto status = c.computeCommand (tnc::Pose2D{0.0, 0.0, 0.0}, cmd);
    EXPECT_EQ (status, tnc::ControllerResult::Success);
    // Forward-favoured rollouts should pull vx upward toward the goal.
    EXPECT_GT (cmd.vx, 0.0);
    EXPECT_LE (std::fabs (cmd.vx), p.v_max + 1e-9);
}

TEST (MecanumMppi, GoalReachedReturnsZeroCommand) {
    tnm::MppiParams p;
    p.horizon = 5;
    p.num_samples = 16;
    p.goal_checker.xy_tolerance  = 0.10;
    p.goal_checker.yaw_tolerance = 0.20;
    tnm::MecanumMppiController c (p);

    tnc::Path2D path;
    path.poses.push_back ({0.0, 0.0, 0.0});
    path.poses.push_back ({0.5, 0.0, 0.0});
    path.poses.push_back ({1.0, 0.0, 0.0});
    c.setPlan (path);

    tnc::Twist2D cmd;
    const auto status = c.computeCommand (tnc::Pose2D{1.0, 0.0, 0.0}, cmd);
    EXPECT_EQ (status, tnc::ControllerResult::GoalReached);
    EXPECT_DOUBLE_EQ (cmd.vx, 0.0);
    EXPECT_DOUBLE_EQ (cmd.vy, 0.0);
    EXPECT_DOUBLE_EQ (cmd.wz, 0.0);
}
