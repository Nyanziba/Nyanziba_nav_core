// pybind11 entry point for the texnitis_nav_core Python bindings.
//
// Exposes the core types, planners, controllers and (when MPPI is
// enabled) the mecanum MPPI controller. Each binding is implemented in
// a separate translation unit so this file stays a registration table.

#include <pybind11/pybind11.h>

namespace py = pybind11;

void bindTypes (py::module_ &);
void bindGridMapView (py::module_ &);
void bindPlanners (py::module_ &);
void bindControllers (py::module_ &);

#ifdef TEXNITIS_NAV_CORE_WITH_MPPI
void bindMppi (py::module_ &);
#endif

PYBIND11_MODULE (_core, m) {
    m.doc () = "texnitis_nav_core: ROS-independent navigation algorithms";

    bindTypes (m);
    bindGridMapView (m);
    bindPlanners (m);
    bindControllers (m);
#ifdef TEXNITIS_NAV_CORE_WITH_MPPI
    bindMppi (m);
#endif
}
