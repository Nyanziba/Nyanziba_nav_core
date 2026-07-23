// Binding: global planners and motion models.

#include <texnitis_nav_core/motion_models/differential_drive_motion_model.hpp>
#include <texnitis_nav_core/motion_models/holonomic_motion_model.hpp>
#include <texnitis_nav_core/planners/astar_params.hpp>
#include <texnitis_nav_core/planners/astar_planner.hpp>
#include <texnitis_nav_core/planners/kinematic_time_astar_planner.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py  = pybind11;
namespace tnc = texnitis::nav_core;

void bindPlanners (py::module_ &m) {
    py::class_<tnc::MotionModel, std::shared_ptr<tnc::MotionModel>> (m, "MotionModel");

    py::class_<tnc::HolonomicMotionModelParams> (m, "HolonomicMotionModelParams")
        .def (py::init<> ())
        .def_readwrite ("primitive_length", &tnc::HolonomicMotionModelParams::primitive_length)
        .def_readwrite ("heading_bins", &tnc::HolonomicMotionModelParams::heading_bins)
        .def_readwrite ("max_velocity_x", &tnc::HolonomicMotionModelParams::max_velocity_x)
        .def_readwrite ("max_velocity_y", &tnc::HolonomicMotionModelParams::max_velocity_y)
        .def_readwrite ("max_acceleration_x", &tnc::HolonomicMotionModelParams::max_acceleration_x)
        .def_readwrite ("max_acceleration_y", &tnc::HolonomicMotionModelParams::max_acceleration_y)
        .def_readwrite ("max_angular_velocity", &tnc::HolonomicMotionModelParams::max_angular_velocity)
        .def_readwrite ("max_angular_acceleration", &tnc::HolonomicMotionModelParams::max_angular_acceleration);
    py::class_<tnc::HolonomicMotionModel, tnc::MotionModel, std::shared_ptr<tnc::HolonomicMotionModel>> (m, "HolonomicMotionModel")
        .def (py::init<tnc::HolonomicMotionModelParams> (), py::arg ("params") = tnc::HolonomicMotionModelParams{})
        .def ("valid", &tnc::HolonomicMotionModel::valid);

    py::class_<tnc::DifferentialDriveMotionModelParams> (m, "DifferentialDriveMotionModelParams")
        .def (py::init<> ())
        .def_readwrite ("primitive_length", &tnc::DifferentialDriveMotionModelParams::primitive_length)
        .def_readwrite ("heading_bins", &tnc::DifferentialDriveMotionModelParams::heading_bins)
        .def_readwrite ("max_linear_velocity", &tnc::DifferentialDriveMotionModelParams::max_linear_velocity)
        .def_readwrite ("max_linear_acceleration", &tnc::DifferentialDriveMotionModelParams::max_linear_acceleration)
        .def_readwrite ("max_angular_velocity", &tnc::DifferentialDriveMotionModelParams::max_angular_velocity)
        .def_readwrite ("max_angular_acceleration", &tnc::DifferentialDriveMotionModelParams::max_angular_acceleration)
        .def_readwrite ("minimum_turning_radius", &tnc::DifferentialDriveMotionModelParams::minimum_turning_radius)
        .def_readwrite ("allow_in_place_rotation", &tnc::DifferentialDriveMotionModelParams::allow_in_place_rotation)
        .def_readwrite ("allow_reverse", &tnc::DifferentialDriveMotionModelParams::allow_reverse);
    py::class_<tnc::DifferentialDriveMotionModel, tnc::MotionModel, std::shared_ptr<tnc::DifferentialDriveMotionModel>> (m, "DifferentialDriveMotionModel")
        .def (py::init<tnc::DifferentialDriveMotionModelParams> (), py::arg ("params") = tnc::DifferentialDriveMotionModelParams{})
        .def ("valid", &tnc::DifferentialDriveMotionModel::valid);

    py::class_<tnc::MapPreprocessorParams> (m, "MapPreprocessorParams")
        .def (py::init<> ())
        .def_readwrite ("occupied_threshold", &tnc::MapPreprocessorParams::occupied_threshold)
        .def_readwrite ("unknown_is_obstacle", &tnc::MapPreprocessorParams::unknown_is_obstacle)
        .def_readwrite ("inflation_radius", &tnc::MapPreprocessorParams::inflation_radius)
        .def_readwrite ("max_step_height", &tnc::MapPreprocessorParams::max_step_height)
        .def_readwrite ("max_slope", &tnc::MapPreprocessorParams::max_slope)
        .def_readwrite ("slope_cost_scale", &tnc::MapPreprocessorParams::slope_cost_scale);
    py::class_<tnc::KinematicTimeAStarParams> (m, "KinematicTimeAStarParams")
        .def (py::init<> ())
        .def_readwrite ("map_preprocessor", &tnc::KinematicTimeAStarParams::map_preprocessor)
        .def_readwrite ("heading_bins", &tnc::KinematicTimeAStarParams::heading_bins)
        .def_readwrite ("max_iterations", &tnc::KinematicTimeAStarParams::max_iterations)
        .def_readwrite ("planning_timeout", &tnc::KinematicTimeAStarParams::planning_timeout)
        .def_readwrite ("heuristic_weight", &tnc::KinematicTimeAStarParams::heuristic_weight)
        .def_readwrite ("goal_yaw_tolerance", &tnc::KinematicTimeAStarParams::goal_yaw_tolerance)
        .def_readwrite ("terrain_vehicle", &tnc::KinematicTimeAStarParams::terrain_vehicle);
    py::class_<tnc::TerrainVehicleParams> (m, "TerrainVehicleParams")
        .def (py::init<> ())
        .def_readwrite ("uphill_speed_scale_per_grade", &tnc::TerrainVehicleParams::uphill_speed_scale_per_grade)
        .def_readwrite ("downhill_speed_scale_per_grade", &tnc::TerrainVehicleParams::downhill_speed_scale_per_grade)
        .def_readwrite ("minimum_speed_scale", &tnc::TerrainVehicleParams::minimum_speed_scale)
        .def_readwrite ("maximum_longitudinal_grade", &tnc::TerrainVehicleParams::maximum_longitudinal_grade)
        .def_readwrite ("track_width", &tnc::TerrainVehicleParams::track_width)
        .def_readwrite ("center_of_gravity_height", &tnc::TerrainVehicleParams::center_of_gravity_height)
        .def_readwrite ("rollover_safety_factor", &tnc::TerrainVehicleParams::rollover_safety_factor)
        .def_readwrite ("require_valid_slope", &tnc::TerrainVehicleParams::require_valid_slope);
    py::class_<tnc::KinematicTimeAStarPlanner> (m, "KinematicTimeAStarPlanner")
        .def (py::init<tnc::KinematicTimeAStarParams, std::shared_ptr<const tnc::MotionModel>> ())
        .def (
            "plan_path",
            [] (tnc::KinematicTimeAStarPlanner &self, const tnc::GridMapView &map, const tnc::Pose2D &start, const tnc::Pose2D &goal) {
                tnc::Path2D path;
                const auto  result = self.planPath (map, start, goal, path);
                return py::make_tuple (result, path, self.lastPlanCost ());
            })
        .def (
            "plan_trajectory",
            [] (tnc::KinematicTimeAStarPlanner &self, const tnc::GridMapView &map, const tnc::Pose2D &start, const tnc::Pose2D &goal) {
                tnc::Trajectory2D trajectory;
                const auto result = self.planTrajectory (map, start, goal, trajectory);
                return py::make_tuple (result, trajectory, self.lastPlanCost ());
            })
        .def ("cancel", &tnc::KinematicTimeAStarPlanner::cancel)
        .def ("reset", &tnc::KinematicTimeAStarPlanner::reset);

    py::class_<tnc::AStarParams> (m, "AStarParams")
        .def (py::init<> ())
        .def_readwrite ("occupied_threshold", &tnc::AStarParams::occupied_threshold)
        .def_readwrite ("unknown_is_obstacle", &tnc::AStarParams::unknown_is_obstacle)
        .def_readwrite ("inflation_radius", &tnc::AStarParams::inflation_radius)
        .def_readwrite ("max_iterations", &tnc::AStarParams::max_iterations)
        .def_readwrite ("heuristic_weight", &tnc::AStarParams::heuristic_weight)
        .def_readwrite ("allow_diagonal", &tnc::AStarParams::allow_diagonal);

    py::class_<tnc::AStarPlanner> (m, "AStarPlanner")
        .def (py::init<> ())
        .def (py::init<const tnc::AStarParams &> ())
        .def ("set_params", &tnc::AStarPlanner::setParams)
        .def (
            "plan_path",
            [] (tnc::AStarPlanner &self, const tnc::GridMapView &map, const tnc::Pose2D &start, const tnc::Pose2D &goal) {
                tnc::Path2D path;
                const auto  status = self.planPath (map, start, goal, path);
                return py::make_tuple (status, path);
            },
            py::arg ("map"), py::arg ("start"), py::arg ("goal"))
        .def ("cancel", &tnc::AStarPlanner::cancel)
        .def ("reset", &tnc::AStarPlanner::reset);

}
