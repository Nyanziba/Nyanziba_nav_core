// Binding: AStarPlanner / HeightAwareAStarPlanner.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <texnitis_nav_core/height_provider.hpp>
#include <texnitis_nav_core/planners/astar_params.hpp>
#include <texnitis_nav_core/planners/astar_planner.hpp>
#include <texnitis_nav_core/planners/height_aware_astar_params.hpp>
#include <texnitis_nav_core/planners/height_aware_astar_planner.hpp>

namespace py = pybind11;
namespace tnc = texnitis::nav_core;

namespace {

/// Trampoline so Python subclasses can implement HeightProvider.
class PyHeightProvider : public tnc::HeightProvider {
   public:
    using tnc::HeightProvider::HeightProvider;

    [[nodiscard]] bool getLatest (tnc::HeightGridView &out_view) const override {
        PYBIND11_OVERRIDE_PURE (bool, tnc::HeightProvider, getLatest, out_view);
    }
};

}  // namespace

void bindPlanners (py::module_ &m) {
    py::class_<tnc::AStarParams> (m, "AStarParams")
        .def (py::init<> ())
        .def_readwrite ("occupied_threshold",  &tnc::AStarParams::occupied_threshold)
        .def_readwrite ("unknown_is_obstacle", &tnc::AStarParams::unknown_is_obstacle)
        .def_readwrite ("inflation_radius",    &tnc::AStarParams::inflation_radius)
        .def_readwrite ("max_iterations",      &tnc::AStarParams::max_iterations)
        .def_readwrite ("heuristic_weight",    &tnc::AStarParams::heuristic_weight)
        .def_readwrite ("allow_diagonal",      &tnc::AStarParams::allow_diagonal);

    py::class_<tnc::AStarPlanner> (m, "AStarPlanner")
        .def (py::init<> ())
        .def (py::init<const tnc::AStarParams &> ())
        .def ("set_params", &tnc::AStarPlanner::setParams)
        .def ("plan_path",
              [] (tnc::AStarPlanner &self, const tnc::GridMapView &map,
                  const tnc::Pose2D &start, const tnc::Pose2D &goal) {
                  tnc::Path2D path;
                  const auto status = self.planPath (map, start, goal, path);
                  return py::make_tuple (status, path);
              },
              py::arg ("map"), py::arg ("start"), py::arg ("goal"))
        .def ("cancel", &tnc::AStarPlanner::cancel)
        .def ("reset", &tnc::AStarPlanner::reset);

    py::class_<tnc::HeightProvider, PyHeightProvider> (m, "HeightProvider")
        .def (py::init<> ())
        .def ("get_latest", &tnc::HeightProvider::getLatest);

    py::class_<tnc::HeightAwareAStarParams> (m, "HeightAwareAStarParams")
        .def (py::init<> ())
        .def_readwrite ("astar",                   &tnc::HeightAwareAStarParams::astar)
        .def_readwrite ("height_lethal_threshold", &tnc::HeightAwareAStarParams::height_lethal_threshold)
        .def_readwrite ("require_height_grid",     &tnc::HeightAwareAStarParams::require_height_grid);

    py::class_<tnc::HeightAwareAStarPlanner> (m, "HeightAwareAStarPlanner")
        .def (py::init<> ())
        .def (py::init<const tnc::HeightAwareAStarParams &, const tnc::HeightProvider *> (),
              py::arg ("params"), py::arg ("provider") = nullptr)
        .def ("set_params", &tnc::HeightAwareAStarPlanner::setParams)
        .def ("set_height_provider", &tnc::HeightAwareAStarPlanner::setHeightProvider,
              py::keep_alive<1, 2> ())
        .def ("plan_path",
              [] (tnc::HeightAwareAStarPlanner &self, const tnc::GridMapView &map,
                  const tnc::Pose2D &start, const tnc::Pose2D &goal) {
                  tnc::Path2D path;
                  const auto status = self.planPath (map, start, goal, path);
                  return py::make_tuple (status, path);
              },
              py::arg ("map"), py::arg ("start"), py::arg ("goal"))
        .def ("cancel", &tnc::HeightAwareAStarPlanner::cancel)
        .def ("reset", &tnc::HeightAwareAStarPlanner::reset);
}
