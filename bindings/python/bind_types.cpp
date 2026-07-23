// Binding: POD types and helpers.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <texnitis_nav_core/result.hpp>
#include <texnitis_nav_core/types.hpp>

namespace py = pybind11;
namespace tnc = texnitis::nav_core;

void bindTypes (py::module_ &m) {
    py::class_<tnc::Pose2D> (m, "Pose2D")
        .def (py::init<> ())
        .def (py::init<double, double, double> (), py::arg ("x"), py::arg ("y"), py::arg ("yaw"))
        .def_readwrite ("x", &tnc::Pose2D::x)
        .def_readwrite ("y", &tnc::Pose2D::y)
        .def_readwrite ("yaw", &tnc::Pose2D::yaw)
        .def ("__repr__", [] (const tnc::Pose2D &p) {
            return "Pose2D(" + std::to_string (p.x) + ", " +
                   std::to_string (p.y) + ", " + std::to_string (p.yaw) + ")";
        });

    py::class_<tnc::Twist2D> (m, "Twist2D")
        .def (py::init<> ())
        .def (py::init<double, double, double> (), py::arg ("vx"), py::arg ("vy"), py::arg ("wz"))
        .def_readwrite ("vx", &tnc::Twist2D::vx)
        .def_readwrite ("vy", &tnc::Twist2D::vy)
        .def_readwrite ("wz", &tnc::Twist2D::wz)
        .def ("__repr__", [] (const tnc::Twist2D &t) {
            return "Twist2D(" + std::to_string (t.vx) + ", " +
                   std::to_string (t.vy) + ", " + std::to_string (t.wz) + ")";
        });

    py::class_<tnc::Path2D> (m, "Path2D")
        .def (py::init<> ())
        .def_readwrite ("poses", &tnc::Path2D::poses)
        .def ("__len__", [] (const tnc::Path2D &p) { return p.poses.size (); });

    py::class_<tnc::TrajectoryPoint2D> (m, "TrajectoryPoint2D")
        .def (py::init<> ())
        .def_readwrite ("pose", &tnc::TrajectoryPoint2D::pose)
        .def_readwrite ("velocity", &tnc::TrajectoryPoint2D::velocity)
        .def_readwrite ("acceleration", &tnc::TrajectoryPoint2D::acceleration)
        .def_readwrite ("time_from_start", &tnc::TrajectoryPoint2D::time_from_start);

    py::class_<tnc::Trajectory2D> (m, "Trajectory2D")
        .def (py::init<> ())
        .def_readwrite ("points", &tnc::Trajectory2D::points)
        .def ("duration", &tnc::Trajectory2D::duration)
        .def ("__len__", [] (const tnc::Trajectory2D &t) { return t.points.size (); });

    m.def ("normalize_angle", &tnc::normalizeAngle);
    m.def ("clamp", &tnc::clamp, py::arg ("value"), py::arg ("lo"), py::arg ("hi"));
    m.def ("distance_xy", &tnc::distanceXY);
    m.def ("shortest_angular_diff", &tnc::shortestAngularDiff);

    py::enum_<tnc::PlanResult> (m, "PlanResult")
        .value ("Success",          tnc::PlanResult::Success)
        .value ("StartOutOfBounds", tnc::PlanResult::StartOutOfBounds)
        .value ("GoalOutOfBounds",  tnc::PlanResult::GoalOutOfBounds)
        .value ("StartInCollision", tnc::PlanResult::StartInCollision)
        .value ("GoalInCollision",  tnc::PlanResult::GoalInCollision)
        .value ("NoPathFound",      tnc::PlanResult::NoPathFound)
        .value ("InvalidMap",       tnc::PlanResult::InvalidMap)
        .value ("Cancelled",        tnc::PlanResult::Cancelled)
        .value ("NotInitialized",   tnc::PlanResult::NotInitialized)
        .value ("InternalError",    tnc::PlanResult::InternalError);

    py::enum_<tnc::ControllerResult> (m, "ControllerResult")
        .value ("Success",             tnc::ControllerResult::Success)
        .value ("GoalReached",         tnc::ControllerResult::GoalReached)
        .value ("EmptyPath",           tnc::ControllerResult::EmptyPath)
        .value ("PathTooFarFromRobot", tnc::ControllerResult::PathTooFarFromRobot)
        .value ("ComputationFailed",   tnc::ControllerResult::ComputationFailed)
        .value ("Cancelled",           tnc::ControllerResult::Cancelled)
        .value ("NotInitialized",      tnc::ControllerResult::NotInitialized)
        .value ("InternalError",       tnc::ControllerResult::InternalError);
}
