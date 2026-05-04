// Smoke tests for the M1 header-only core. These verify that the public
// types compile, link and behave at their boundary conditions. Heavier
// algorithmic tests (A*, Pure Pursuit, MPPI) come in later milestones.

#include "texnitis_nav_core/grid_map_view.hpp"
#include "texnitis_nav_core/logger.hpp"
#include "texnitis_nav_core/result.hpp"
#include "texnitis_nav_core/types.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

namespace tnc = texnitis::nav_core;

TEST (Types, Pose2DDefaultsAreZero) {
    tnc::Pose2D p;
    EXPECT_DOUBLE_EQ (p.x, 0.0);
    EXPECT_DOUBLE_EQ (p.y, 0.0);
    EXPECT_DOUBLE_EQ (p.yaw, 0.0);
}

TEST (Types, Twist2DDefaultsAreZero) {
    tnc::Twist2D v;
    EXPECT_DOUBLE_EQ (v.vx, 0.0);
    EXPECT_DOUBLE_EQ (v.vy, 0.0);
    EXPECT_DOUBLE_EQ (v.wz, 0.0);
}

TEST (Types, NormalizeAngleWrapsBothDirections) {
    EXPECT_NEAR (tnc::normalizeAngle (3.0 * tnc::kPi), tnc::kPi, 1e-12);
    EXPECT_NEAR (tnc::normalizeAngle (-3.0 * tnc::kPi), -tnc::kPi, 1e-12);
    EXPECT_DOUBLE_EQ (tnc::normalizeAngle (0.0), 0.0);
}

TEST (Types, ClampHonoursBounds) {
    EXPECT_DOUBLE_EQ (tnc::clamp (5.0, 0.0, 10.0), 5.0);
    EXPECT_DOUBLE_EQ (tnc::clamp (-1.0, 0.0, 10.0), 0.0);
    EXPECT_DOUBLE_EQ (tnc::clamp (11.0, 0.0, 10.0), 10.0);
}

TEST (Types, DistanceXYIsHypot) {
    tnc::Pose2D a{0.0, 0.0, 0.0};
    tnc::Pose2D b{3.0, 4.0, 1.5};
    EXPECT_DOUBLE_EQ (tnc::distanceXY (a, b), 5.0);
}

TEST (Types, ShortestAngularDiffNeverExceedsPi) {
    EXPECT_NEAR (tnc::shortestAngularDiff (0.1, -0.1), -0.2, 1e-12);
    // 350 deg -> 10 deg should be +20 deg (the shortest way), not -340 deg.
    const double from = -3.0;          // close to -pi side
    const double to   = 3.0;           // close to +pi side
    const double diff = tnc::shortestAngularDiff (from, to);
    EXPECT_LE (std::fabs (diff), tnc::kPi);
}

TEST (GridMapView, AtReturnsOccupiedWhenNullData) {
    tnc::GridMapView m;
    m.width = 10;
    m.height = 10;
    EXPECT_EQ (m.at (0, 0), 100);
    EXPECT_FALSE (m.inBounds (-1, 0));
    EXPECT_FALSE (m.inBounds (10, 0));
}

TEST (GridMapView, WorldToMapRoundTrip) {
    std::vector<int8_t> buf (16, 0);
    tnc::GridMapView m;
    m.data = buf.data ();
    m.width = 4;
    m.height = 4;
    m.resolution = 0.5;
    m.origin_x = -1.0;
    m.origin_y = -1.0;

    int mx = -99;
    int my = -99;
    EXPECT_TRUE (m.worldToMap (0.0, 0.0, mx, my));
    EXPECT_EQ (mx, 2);
    EXPECT_EQ (my, 2);

    double wx = 0.0;
    double wy = 0.0;
    m.mapToWorld (mx, my, wx, wy);
    // mapToWorld returns the cell centre, so the round-trip lands in the
    // same cell as the original world coord.
    int rx = -99;
    int ry = -99;
    EXPECT_TRUE (m.worldToMap (wx, wy, rx, ry));
    EXPECT_EQ (rx, mx);
    EXPECT_EQ (ry, my);
}

TEST (GridMapView, AtWorldClampsOutOfBoundsToOccupied) {
    std::vector<int8_t> buf{0, 0, 0, 0};
    tnc::GridMapView m;
    m.data = buf.data ();
    m.width = 2;
    m.height = 2;
    m.resolution = 1.0;
    m.origin_x = 0.0;
    m.origin_y = 0.0;
    EXPECT_EQ (m.atWorld (0.5, 0.5), 0);
    EXPECT_EQ (m.atWorld (10.0, 0.5), 100);
    EXPECT_EQ (m.atWorld (-1.0, 0.5), 100);
}

TEST (Result, ToStringCoversAllValues) {
    EXPECT_STREQ (tnc::toString (tnc::PlanResult::Success), "Success");
    EXPECT_STREQ (tnc::toString (tnc::PlanResult::NoPathFound), "NoPathFound");
    EXPECT_STREQ (tnc::toString (tnc::PlanResult::NotInitialized), "NotInitialized");
    EXPECT_STREQ (tnc::toString (tnc::ControllerResult::Success), "Success");
    EXPECT_STREQ (tnc::toString (tnc::ControllerResult::EmptyPath), "EmptyPath");
    EXPECT_STREQ (tnc::toString (tnc::ControllerResult::Cancelled), "Cancelled");
}

TEST (Logger, DefaultStderrLoggerDoesNotThrow) {
    auto logger = tnc::defaultStderrLogger ();
    EXPECT_NO_THROW (logger (tnc::LogLevel::Info, std::string_view{"smoke message"}));
}

TEST (Logger, NullLoggerIsSafeViaHelper) {
    tnc::LoggerFn empty;
    EXPECT_NO_THROW (tnc::log (empty, tnc::LogLevel::Warn, "no-op"));
}
