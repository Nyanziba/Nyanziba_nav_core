#pragma once

#include "texnitis_nav_core/map_processing/map_preprocessor.hpp"
#include "texnitis_nav_core/motion_models/motion_model.hpp"
#include "texnitis_nav_core/planner_base.hpp"
#include "texnitis_nav_core/terrain/terrain_traversal_evaluator.hpp"

#include <atomic>
#include <memory>

namespace texnitis::nav_core {

struct KinematicTimeAStarParams {
    MapPreprocessorParams map_preprocessor{};
    int                   heading_bins{16};
    int                   max_iterations{0};
    double                planning_timeout{1.0};
    double                heuristic_weight{1.0};
    double                goal_position_tolerance{0.10};
    double                goal_yaw_tolerance{0.20};
    double                traversal_cost_weight{1.0};
    TerrainVehicleParams  terrain_vehicle{};
};

class KinematicTimeAStarPlanner final : public PlannerBase {
   public:
    KinematicTimeAStarPlanner (KinematicTimeAStarParams params, std::shared_ptr<const MotionModel> motion_model);

    [[nodiscard]] PlanResult planPath (const GridMapView &map, const Pose2D &start, const Pose2D &goal, Path2D &out_path) override;
    [[nodiscard]] PlanResult planPathWithTerrain (const GridMapView &map, const TerrainLayersView &terrain, const Pose2D &start, const Pose2D &goal, Path2D &out_path);
    [[nodiscard]] PlanResult planTrajectory (const GridMapView &map, const Pose2D &start, const Pose2D &goal, Trajectory2D &out_trajectory);
    [[nodiscard]] PlanResult planTrajectoryWithTerrain (const GridMapView &map, const TerrainLayersView &terrain, const Pose2D &start, const Pose2D &goal, Trajectory2D &out_trajectory);
    void                     cancel () noexcept override {
        cancel_flag_.store (true, std::memory_order_relaxed);
    }
    void reset () override {
        cancel_flag_.store (false, std::memory_order_relaxed);
    }
    [[nodiscard]] double lastPlanCost () const noexcept {
        return last_plan_cost_;
    }

   private:
    KinematicTimeAStarParams           params_;
    std::shared_ptr<const MotionModel> motion_model_;
    MapPreprocessor                    preprocessor_;
    TerrainTraversalEvaluator          terrain_evaluator_;
    ProcessedPlanningMapStorage        processed_storage_;
    std::atomic<bool>                  cancel_flag_{false};
    double                             last_plan_cost_{0.0};
};

}  // namespace texnitis::nav_core
