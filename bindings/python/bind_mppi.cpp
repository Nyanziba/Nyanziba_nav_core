// Binding: MPPI controller (only when NAV_CORE_WITH_MPPI=ON).

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include <texnitis_nav_core/mppi/mecanum_kinematics.hpp>
#include <texnitis_nav_core/mppi/mecanum_mppi_controller.hpp>
#include <texnitis_nav_core/mppi/mppi_params.hpp>

namespace py  = pybind11;
namespace tnc = texnitis::nav_core;
namespace tnm = texnitis::nav_core::mppi;

void bindMppi (py::module_ &m) {
    auto mppi = m.def_submodule ("mppi", "Sample-based MPPI controller and mecanum kinematics");

    py::class_<tnm::MecanumKinematics> (mppi, "MecanumKinematics")
        .def (py::init<double, double, double, double> (),
              py::arg ("v_max"), py::arg ("omega_max"),
              py::arg ("dt"), py::arg ("wheel_base") = 0.30)
        .def ("update_state", &tnm::MecanumKinematics::updateState)
        .def ("wheel_speeds", &tnm::MecanumKinematics::getWheelSpeeds);

    py::class_<tnm::MppiParams> (mppi, "MppiParams")
        .def (py::init<> ())
        .def_readwrite ("horizon",      &tnm::MppiParams::horizon)
        .def_readwrite ("num_samples",  &tnm::MppiParams::num_samples)
        .def_readwrite ("lambda_",      &tnm::MppiParams::lambda)
        .def_readwrite ("sigma",        &tnm::MppiParams::sigma)
        .def_readwrite ("u_max",        &tnm::MppiParams::u_max)
        .def_readwrite ("dt",           &tnm::MppiParams::dt)
        .def_readwrite ("w_track",      &tnm::MppiParams::w_track)
        .def_readwrite ("w_control",    &tnm::MppiParams::w_control)
        .def_readwrite ("w_yaw",        &tnm::MppiParams::w_yaw)
        .def_readwrite ("v_max",        &tnm::MppiParams::v_max)
        .def_readwrite ("omega_max",    &tnm::MppiParams::omega_max)
        .def_readwrite ("wheel_base",   &tnm::MppiParams::wheel_base)
        .def_readwrite ("seed",         &tnm::MppiParams::seed)
        .def_readwrite ("goal_checker", &tnm::MppiParams::goal_checker);

    py::class_<tnm::MecanumMppiController> (mppi, "MecanumMppiController")
        .def (py::init<> ())
        .def (py::init<const tnm::MppiParams &> ())
        .def ("set_params", &tnm::MecanumMppiController::setParams)
        .def ("set_plan",   &tnm::MecanumMppiController::setPlan)
        .def ("compute_command",
              [] (tnm::MecanumMppiController &self, const tnc::Pose2D &pose) {
                  tnc::Twist2D out;
                  const auto status = self.computeCommand (pose, out);
                  return py::make_tuple (status, out);
              })
        // GIL 解放はコールバック中に CPython 内部状態を踏むケースで
        // SEGV を起こすことが macOS で観測されたため、現状は外している。
        // CasADi 統合と合わせて再評価する予定。
        .def ("is_goal_reached", &tnm::MecanumMppiController::isGoalReached)
        .def ("cancel", &tnm::MecanumMppiController::cancel)
        .def ("reset",  &tnm::MecanumMppiController::reset);
}
