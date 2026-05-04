// M4 smoke tests: HeightAwareAStarPlanner (with HeightProvider) and
// DiffDrivePurePursuitController.

#include "texnitis_nav_core/controllers/diff_drive_pure_pursuit_controller.hpp"
#include "texnitis_nav_core/grid_map_view.hpp"
#include "texnitis_nav_core/height_provider.hpp"
#include "texnitis_nav_core/planners/height_aware_astar_planner.hpp"
#include "texnitis_nav_core/types.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace tnc = texnitis::nav_core;

namespace {

class StubHeightProvider final : public tnc::HeightProvider {
   public:
    StubHeightProvider () = default;

    void set (const std::vector<int8_t> &data,
              int                        width,
              int                        height,
              double                     resolution = 0.1,
              double                     origin_x   = 0.0,
              double                     origin_y   = 0.0) {
        data_       = data;
        width_      = width;
        height_     = height;
        resolution_ = resolution;
        origin_x_   = origin_x;
        origin_y_   = origin_y;
        ready_      = true;
    }

    [[nodiscard]] bool getLatest (tnc::HeightGridView &out_view) const override {
        if (!ready_) {
            return false;
        }
        out_view.data       = data_.data ();
        out_view.width      = width_;
        out_view.height     = height_;
        out_view.resolution = resolution_;
        out_view.origin_x   = origin_x_;
        out_view.origin_y   = origin_y_;
        return true;
    }

   private:
    std::vector<int8_t> data_;
    int                 width_{0};
    int                 height_{0};
    double              resolution_{0.1};
    double              origin_x_{0.0};
    double              origin_y_{0.0};
    bool                ready_{false};
};

}  // namespace

TEST (HeightAwareAStar, NotInitialisedWithoutHeightGrid) {
    std::vector<int8_t> grid (100, 0);
    tnc::GridMapView m;
    m.data       = grid.data ();
    m.width      = 10;
    m.height     = 10;
    m.resolution = 0.1;

    tnc::HeightAwareAStarParams params;
    params.require_height_grid = true;
    tnc::HeightAwareAStarPlanner planner (params, /*provider=*/nullptr);
    tnc::Path2D path;
    EXPECT_EQ (planner.planPath (m, {}, {0.5, 0.5, 0.0}, path),
               tnc::PlanResult::NotInitialized);
}

TEST (HeightAwareAStar, MergesHeightLethalCells) {
    std::vector<int8_t> grid (100, 0);
    tnc::GridMapView m;
    m.data       = grid.data ();
    m.width      = 10;
    m.height     = 10;
    m.resolution = 0.1;

    // Block out the middle row via the height grid (height >= threshold).
    std::vector<int8_t> heights (100, 0);
    for (int x = 0; x < 10; ++x) {
        heights[5 * 10 + x] = 80;
    }
    StubHeightProvider provider;
    provider.set (heights, 10, 10, 0.1, 0.0, 0.0);

    tnc::HeightAwareAStarParams params;
    params.height_lethal_threshold = 50;
    params.astar.inflation_radius  = 0.0;
    tnc::HeightAwareAStarPlanner planner (params, &provider);

    tnc::Path2D path;
    const auto status = planner.planPath (m, {0.05, 0.05, 0.0}, {0.05, 0.95, 0.0}, path);
    // The middle row is fully blocked, no path can cross it on a 10x10
    // map with no escape route on the sides.
    EXPECT_TRUE (status == tnc::PlanResult::NoPathFound);
    EXPECT_TRUE (path.poses.empty ());
}

TEST (HeightAwareAStar, FreeWhenHeightBelowThreshold) {
    std::vector<int8_t> grid (100, 0);
    tnc::GridMapView m;
    m.data       = grid.data ();
    m.width      = 10;
    m.height     = 10;
    m.resolution = 0.1;

    std::vector<int8_t> heights (100, 10);  // all benign
    StubHeightProvider provider;
    provider.set (heights, 10, 10, 0.1, 0.0, 0.0);

    tnc::HeightAwareAStarParams params;
    params.height_lethal_threshold = 50;
    params.astar.inflation_radius  = 0.0;
    tnc::HeightAwareAStarPlanner planner (params, &provider);

    tnc::Path2D path;
    EXPECT_EQ (planner.planPath (m, {0.05, 0.05, 0.0}, {0.85, 0.85, 0.0}, path),
               tnc::PlanResult::Success);
    EXPECT_FALSE (path.poses.empty ());
}

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
    for (int i = 0; i < 20; ++i) {
        path.poses.push_back ({0.1 * i, 0.0, 0.0});
    }
    controller.setPlan (path);

    tnc::Twist2D cmd;
    const auto first = controller.computeCommand (tnc::Pose2D{0.0, 0.0, 0.0}, cmd);
    EXPECT_EQ (first, tnc::ControllerResult::Success);
    // First tick: vx must not exceed max_acceleration * control_dt = 0.05.
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
    path.poses.push_back ({0.0, 0.0, 0.0});
    path.poses.push_back ({0.5, 0.0, 0.0});
    path.poses.push_back ({1.0, 0.0, 0.0});
    controller.setPlan (path);

    tnc::Twist2D cmd;
    const auto status = controller.computeCommand (tnc::Pose2D{1.0, 0.0, 0.0}, cmd);
    EXPECT_EQ (status, tnc::ControllerResult::GoalReached);
    EXPECT_DOUBLE_EQ (cmd.vx, 0.0);
    EXPECT_TRUE (controller.isGoalReached (params.goal_checker.xy_tolerance,
                                           params.goal_checker.yaw_tolerance));
}
