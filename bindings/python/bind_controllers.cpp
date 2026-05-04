// Binding: Lookahead and DiffDrivePurePursuit controllers.

#include <pybind11/pybind11.h>

#include <texnitis_nav_core/controllers/diff_drive_pure_pursuit_controller.hpp>
#include <texnitis_nav_core/controllers/diff_drive_pure_pursuit_params.hpp>
#include <texnitis_nav_core/controllers/lookahead_controller.hpp>
#include <texnitis_nav_core/controllers/lookahead_params.hpp>
#include <texnitis_nav_core/goal_checker.hpp>

namespace py = pybind11;
namespace tnc = texnitis::nav_core;

void bindControllers (py::module_ &m) {
    py::class_<tnc::GoalCheckerParams> (m, "GoalCheckerParams")
        .def (py::init<> ())
        .def_readwrite ("xy_tolerance",  &tnc::GoalCheckerParams::xy_tolerance)
        .def_readwrite ("yaw_tolerance", &tnc::GoalCheckerParams::yaw_tolerance)
        .def_readwrite ("stateful",      &tnc::GoalCheckerParams::stateful);

    py::class_<tnc::LookaheadParams> (m, "LookaheadParams")
        .def (py::init<> ())
        .def_readwrite ("kp_xy",                  &tnc::LookaheadParams::kp_xy)
        .def_readwrite ("kp_yaw",                 &tnc::LookaheadParams::kp_yaw)
        .def_readwrite ("max_speed_xy",           &tnc::LookaheadParams::max_speed_xy)
        .def_readwrite ("max_speed_yaw",          &tnc::LookaheadParams::max_speed_yaw)
        .def_readwrite ("lookahead_dist",         &tnc::LookaheadParams::lookahead_dist)
        .def_readwrite ("linear_threshold_for_wz", &tnc::LookaheadParams::linear_threshold_for_wz)
        .def_readwrite ("max_wz_when_moving",     &tnc::LookaheadParams::max_wz_when_moving)
        .def_readwrite ("use_diff_drive",         &tnc::LookaheadParams::use_diff_drive)
        .def_readwrite ("goal_checker",           &tnc::LookaheadParams::goal_checker);

    py::class_<tnc::LookaheadController> (m, "LookaheadController")
        .def (py::init<> ())
        .def (py::init<const tnc::LookaheadParams &> ())
        .def ("set_params", &tnc::LookaheadController::setParams)
        .def ("set_plan",   &tnc::LookaheadController::setPlan)
        .def ("compute_command",
              [] (tnc::LookaheadController &self, const tnc::Pose2D &pose) {
                  tnc::Twist2D out;
                  const auto status = self.computeCommand (pose, out);
                  return py::make_tuple (status, out);
              })
        .def ("is_goal_reached", &tnc::LookaheadController::isGoalReached)
        .def ("cancel", &tnc::LookaheadController::cancel)
        .def ("reset",  &tnc::LookaheadController::reset);

    py::class_<tnc::DiffDrivePurePursuitParams> (m, "DiffDrivePurePursuitParams")
        .def (py::init<> ())
        .def_readwrite ("max_linear_velocity",       &tnc::DiffDrivePurePursuitParams::max_linear_velocity)
        .def_readwrite ("min_linear_velocity",       &tnc::DiffDrivePurePursuitParams::min_linear_velocity)
        .def_readwrite ("max_angular_velocity",      &tnc::DiffDrivePurePursuitParams::max_angular_velocity)
        .def_readwrite ("max_acceleration",          &tnc::DiffDrivePurePursuitParams::max_acceleration)
        .def_readwrite ("max_angular_acceleration",  &tnc::DiffDrivePurePursuitParams::max_angular_acceleration)
        .def_readwrite ("lookahead_time",            &tnc::DiffDrivePurePursuitParams::lookahead_time)
        .def_readwrite ("min_lookahead_distance",    &tnc::DiffDrivePurePursuitParams::min_lookahead_distance)
        .def_readwrite ("max_lookahead_distance",    &tnc::DiffDrivePurePursuitParams::max_lookahead_distance)
        .def_readwrite ("angle_speed_p",             &tnc::DiffDrivePurePursuitParams::angle_speed_p)
        .def_readwrite ("control_dt",                &tnc::DiffDrivePurePursuitParams::control_dt)
        .def_readwrite ("goal_checker",              &tnc::DiffDrivePurePursuitParams::goal_checker);

    py::class_<tnc::DiffDrivePurePursuitController> (m, "DiffDrivePurePursuitController")
        .def (py::init<> ())
        .def (py::init<const tnc::DiffDrivePurePursuitParams &> ())
        .def ("set_params", &tnc::DiffDrivePurePursuitController::setParams)
        .def ("set_plan",   &tnc::DiffDrivePurePursuitController::setPlan)
        .def ("compute_command",
              [] (tnc::DiffDrivePurePursuitController &self, const tnc::Pose2D &pose) {
                  tnc::Twist2D out;
                  const auto status = self.computeCommand (pose, out);
                  return py::make_tuple (status, out);
              })
        .def ("is_goal_reached", &tnc::DiffDrivePurePursuitController::isGoalReached)
        .def ("cancel", &tnc::DiffDrivePurePursuitController::cancel)
        .def ("reset",  &tnc::DiffDrivePurePursuitController::reset);
}
