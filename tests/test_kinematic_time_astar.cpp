#include "texnitis_nav_core/motion_models/differential_drive_motion_model.hpp"
#include "texnitis_nav_core/motion_models/holonomic_motion_model.hpp"
#include "texnitis_nav_core/planners/kinematic_time_astar_planner.hpp"
#include "texnitis_nav_core/terrain/terrain_traversal_evaluator.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace nc = texnitis::nav_core;

namespace {
nc::GridMapView openMap (std::vector<int8_t> &data) {
    data.assign (20 * 20, 0);
    return {data.data (), 20, 20, 0.1, 0.0, 0.0};
}
}  // namespace

TEST (KinematicTimeAStar, PreservesRequestedStartAndGoalPose) {
    std::vector<int8_t>          data;
    auto                         map   = openMap (data);
    auto                         model = std::make_shared<nc::HolonomicMotionModel> ();
    nc::KinematicTimeAStarParams params;
    params.map_preprocessor.inflation_radius = 0.0;
    nc::KinematicTimeAStarPlanner planner (params, model);
    nc::Path2D                    path;
    const nc::Pose2D              start{0.15, 0.15, 0.0};
    const nc::Pose2D              goal{1.45, 1.25, nc::kPi / 2.0};
    ASSERT_EQ (planner.planPath (map, start, goal, path), nc::PlanResult::Success);
    ASSERT_GE (path.poses.size (), 2U);
    EXPECT_DOUBLE_EQ (path.poses.front ().x, start.x);
    EXPECT_DOUBLE_EQ (path.poses.front ().y, start.y);
    EXPECT_NEAR (path.poses.back ().yaw, goal.yaw, 1e-9);
    EXPECT_GT (planner.lastPlanCost (), 0.0);
}

TEST (KinematicTimeAStar, RejectsGoalOutsideMap) {
    std::vector<int8_t>           data;
    auto                          map = openMap (data);
    nc::KinematicTimeAStarPlanner planner ({}, std::make_shared<nc::HolonomicMotionModel> ());
    nc::Path2D                    path;
    EXPECT_EQ (planner.planPath (map, {0.1, 0.1, 0.0}, {9.0, 9.0, 0.0}, path), nc::PlanResult::GoalOutOfBounds);
}

TEST (KinematicTimeAStar, RepeatedPlanningIsDeterministic) {
    std::vector<int8_t>          data;
    auto                         map = openMap (data);
    nc::KinematicTimeAStarParams params;
    params.map_preprocessor.inflation_radius = 0.0;
    nc::KinematicTimeAStarPlanner planner (params, std::make_shared<nc::DifferentialDriveMotionModel> ());
    nc::Path2D                    first, second;
    ASSERT_EQ (planner.planPath (map, {0.15, 0.15, 0.0}, {1.15, 0.15, 0.0}, first), nc::PlanResult::Success);
    ASSERT_EQ (planner.planPath (map, {0.15, 0.15, 0.0}, {1.15, 0.15, 0.0}, second), nc::PlanResult::Success);
    ASSERT_EQ (first.poses.size (), second.poses.size ());
    for (size_t i = 0; i < first.poses.size (); ++i) {
        EXPECT_DOUBLE_EQ (first.poses[i].x, second.poses[i].x);
        EXPECT_DOUBLE_EQ (first.poses[i].y, second.poses[i].y);
        EXPECT_DOUBLE_EQ (first.poses[i].yaw, second.poses[i].yaw);
    }
}

TEST (KinematicTimeAStar, TerrainStepBlocksGoalCell) {
    std::vector<int8_t> data;
    auto                map = openMap (data);
    std::vector<float>  elevation (data.size (), 0.0F);
    elevation[10 * 20 + 10] = 0.20F;
    nc::TerrainLayersView terrain;
    terrain.elevation = nc::ElevationGridView{elevation.data (), 20, 20, 0.1, 0.0, 0.0};
    nc::KinematicTimeAStarParams params;
    params.map_preprocessor.inflation_radius = 0.0;
    params.map_preprocessor.max_step_height  = 0.05;
    nc::KinematicTimeAStarPlanner planner (params, std::make_shared<nc::HolonomicMotionModel> ());
    nc::Path2D                    path;
    EXPECT_EQ (planner.planPathWithTerrain (map, terrain, {0.15, 0.15, 0.0}, {1.05, 1.05, 0.0}, path), nc::PlanResult::GoalInCollision);
}

TEST (KinematicTimeAStar, TrajectoryHasMonotonicTimeAndStopsAtGoal) {
    std::vector<int8_t> data;
    auto                map = openMap (data);
    nc::KinematicTimeAStarParams params;
    params.map_preprocessor.inflation_radius = 0.0;
    nc::KinematicTimeAStarPlanner planner (params, std::make_shared<nc::DifferentialDriveMotionModel> ());
    nc::Trajectory2D trajectory;

    ASSERT_EQ (planner.planTrajectory (map, {0.15, 0.15, 0.0}, {1.15, 0.15, 0.0}, trajectory), nc::PlanResult::Success);
    ASSERT_GE (trajectory.points.size (), 2U);
    EXPECT_DOUBLE_EQ (trajectory.points.front ().time_from_start, 0.0);
    for (size_t i = 1; i < trajectory.points.size (); ++i) {
        EXPECT_GT (trajectory.points[i].time_from_start, trajectory.points[i - 1].time_from_start);
    }
    EXPECT_DOUBLE_EQ (trajectory.points.back ().velocity.vx, 0.0);
    EXPECT_DOUBLE_EQ (trajectory.points.back ().velocity.vy, 0.0);
    EXPECT_DOUBLE_EQ (trajectory.points.back ().velocity.wz, 0.0);
    EXPECT_DOUBLE_EQ (trajectory.duration (), trajectory.points.back ().time_from_start);
}

TEST (TerrainTraversalEvaluator, UphillTravelTakesLongerThanDownhill) {
    nc::TerrainVehicleParams params;
    params.uphill_speed_scale_per_grade   = 2.0;
    params.downhill_speed_scale_per_grade = 0.25;
    nc::TerrainTraversalEvaluator evaluator (params);
    const nc::TerrainTraversalContext east{0.0, 0.0, 0.20, 0.0};
    const nc::TerrainTraversalContext west{nc::kPi, nc::kPi, 0.20, 0.0};

    const auto uphill   = evaluator.evaluate (1.0, east);
    const auto downhill = evaluator.evaluate (1.0, west);

    ASSERT_TRUE (uphill.traversable);
    ASSERT_TRUE (downhill.traversable);
    EXPECT_GT (uphill.travel_time, downhill.travel_time);
}

TEST (TerrainTraversalEvaluator, RejectsLateralSlopeBeyondStaticStabilityLimit) {
    nc::TerrainVehicleParams params;
    params.track_width             = 0.40;
    params.center_of_gravity_height = 0.50;
    params.rollover_safety_factor   = 0.80;
    nc::TerrainTraversalEvaluator evaluator (params);
    const nc::TerrainTraversalContext context{0.0, 0.0, 0.0, 0.50};

    const auto result = evaluator.evaluate (1.0, context);

    EXPECT_FALSE (result.traversable);
    EXPECT_LT (result.rollover_margin, 0.0);
}

TEST (TerrainTraversalEvaluator, InvalidGradientIsRejectedWhenTerrainRequired) {
    nc::TerrainVehicleParams params;
    params.require_valid_slope = true;
    nc::TerrainTraversalEvaluator evaluator (params);
    const nc::TerrainTraversalContext context{0.0, 0.0, NAN, 0.0};

    EXPECT_FALSE (evaluator.evaluate (1.0, context).traversable);
}

TEST (KinematicTimeAStar, RejectsInvalidMapAndClearsPreviousTrajectory) {
    nc::GridMapView invalid;
    nc::Trajectory2D trajectory;
    trajectory.points.push_back ({});
    nc::KinematicTimeAStarPlanner planner ({}, std::make_shared<nc::HolonomicMotionModel> ());

    EXPECT_EQ (planner.planTrajectory (invalid, {}, {1.0, 0.0, 0.0}, trajectory), nc::PlanResult::InvalidMap);
    EXPECT_TRUE (trajectory.points.empty ());
}

TEST (KinematicTimeAStar, RejectsStartOutsideAndStartInCollision) {
    std::vector<int8_t> data;
    auto map = openMap (data);
    nc::KinematicTimeAStarPlanner planner ({}, std::make_shared<nc::HolonomicMotionModel> ());
    nc::Trajectory2D trajectory;
    EXPECT_EQ (planner.planTrajectory (map, {-1.0, 0.1, 0.0}, {1.0, 0.1, 0.0}, trajectory), nc::PlanResult::StartOutOfBounds);
    data[1 * 20 + 1] = 100;
    EXPECT_EQ (planner.planTrajectory (map, {0.15, 0.15, 0.0}, {1.0, 0.1, 0.0}, trajectory), nc::PlanResult::StartInCollision);
}

TEST (KinematicTimeAStar, MissingRequiredSlopeIsNotInitialized) {
    std::vector<int8_t> data;
    auto map = openMap (data);
    nc::KinematicTimeAStarParams params;
    params.map_preprocessor.missing_terrain_policy = nc::MissingTerrainPolicy::RequireSlope;
    nc::KinematicTimeAStarPlanner planner (params, std::make_shared<nc::HolonomicMotionModel> ());
    nc::Trajectory2D trajectory;

    EXPECT_EQ (planner.planTrajectory (map, {0.15, 0.15, 0.0}, {1.0, 0.1, 0.0}, trajectory), nc::PlanResult::NotInitialized);
}

TEST (KinematicTimeAStar, RejectsInvalidPlannerAndMotionModelParameters) {
    std::vector<int8_t> data;
    auto map = openMap (data);
    nc::Trajectory2D trajectory;
    nc::KinematicTimeAStarParams params;
    params.heading_bins = 3;
    nc::KinematicTimeAStarPlanner bad_bins (params, std::make_shared<nc::HolonomicMotionModel> ());
    EXPECT_EQ (bad_bins.planTrajectory (map, {}, {1.0, 0.0, 0.0}, trajectory), nc::PlanResult::NotInitialized);

    nc::HolonomicMotionModelParams model_params;
    model_params.max_velocity_x = 0.0;
    nc::KinematicTimeAStarPlanner bad_model ({}, std::make_shared<nc::HolonomicMotionModel> (model_params));
    EXPECT_EQ (bad_model.planTrajectory (map, {}, {1.0, 0.0, 0.0}, trajectory), nc::PlanResult::NotInitialized);

    nc::KinematicTimeAStarPlanner missing_model ({}, nullptr);
    EXPECT_EQ (missing_model.planTrajectory (map, {}, {1.0, 0.0, 0.0}, trajectory), nc::PlanResult::NotInitialized);
}

TEST (KinematicTimeAStar, ValidSlopeLayerParticipatesInPlanning) {
    std::vector<int8_t> data;
    auto map = openMap (data);
    std::vector<float> gx (data.size (), 0.1F), gy (data.size (), 0.0F);
    nc::TerrainLayersView terrain;
    terrain.slope = nc::SlopeGridView{
        {gx.data (), 20, 20, 0.1, 0.0, 0.0},
        {gy.data (), 20, 20, 0.1, 0.0, 0.0}};
    nc::KinematicTimeAStarParams params;
    params.map_preprocessor.inflation_radius = 0.0;
    nc::KinematicTimeAStarPlanner planner (params, std::make_shared<nc::HolonomicMotionModel> ());
    nc::Trajectory2D trajectory;

    EXPECT_EQ (planner.planTrajectoryWithTerrain (map, terrain, {0.15, 0.15, 0.0}, {1.15, 0.15, 0.0}, trajectory), nc::PlanResult::Success);
    EXPECT_GT (trajectory.duration (), 0.0);
}

TEST (KinematicTimeAStar, IterationLimitReturnsNoPath) {
    std::vector<int8_t> data;
    auto map = openMap (data);
    nc::KinematicTimeAStarParams params;
    params.max_iterations = 1;
    nc::KinematicTimeAStarPlanner planner (params, std::make_shared<nc::HolonomicMotionModel> ());
    nc::Trajectory2D trajectory;

    EXPECT_EQ (planner.planTrajectory (map, {0.15, 0.15, 0.0}, {1.5, 1.5, 0.0}, trajectory), nc::PlanResult::NoPathFound);
}

TEST (TerrainTraversalEvaluator, CoversLongitudinalLimitAndMissingSlopeFallback) {
    nc::TerrainVehicleParams params;
    params.maximum_longitudinal_grade = 0.2;
    nc::TerrainTraversalEvaluator evaluator (params);
    EXPECT_FALSE (evaluator.evaluate (1.0, {0.0, 0.0, 0.3, 0.0}).traversable);
    const auto fallback = evaluator.evaluate (1.0, {0.0, 0.0, 0.0, NAN});
    EXPECT_TRUE (fallback.traversable);
    EXPECT_DOUBLE_EQ (fallback.travel_time, 1.0);
}

TEST (TerrainTraversalEvaluator, ZeroCenterOfGravityHeightRejectsAnyLateralSlope) {
    nc::TerrainVehicleParams params;
    params.center_of_gravity_height = 0.0;
    nc::TerrainTraversalEvaluator evaluator (params);

    EXPECT_FALSE (evaluator.evaluate (1.0, {0.0, 0.0, 0.0, 0.01}).traversable);
}
