// Binding: GridMapView (numpy-friendly).
//
// Python users construct a GridMapView from a numpy.ndarray of dtype
// int8 and shape (height, width). The buffer is borrowed for the
// lifetime of the view, so the caller must keep the underlying array
// alive while the view is in use.

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <texnitis_nav_core/grid_map_view.hpp>

namespace py = pybind11;
namespace tnc = texnitis::nav_core;

void bindGridMapView (py::module_ &m) {
    py::class_<tnc::GridMapView> (m, "GridMapView")
        .def (py::init<> ())
        .def_static (
            "from_numpy",
            [] (py::array_t<int8_t, py::array::c_style | py::array::forcecast> data,
                double resolution, double origin_x, double origin_y) {
                if (data.ndim () != 2) {
                    throw std::runtime_error (
                        "GridMapView.from_numpy expects a 2D array (height, width)");
                }
                tnc::GridMapView view;
                view.data       = data.data ();
                view.height     = static_cast<int> (data.shape (0));
                view.width      = static_cast<int> (data.shape (1));
                view.resolution = resolution;
                view.origin_x   = origin_x;
                view.origin_y   = origin_y;
                return view;
            },
            py::arg ("data"), py::arg ("resolution") = 0.05,
            py::arg ("origin_x") = 0.0, py::arg ("origin_y") = 0.0,
            py::keep_alive<0, 1> ())
        .def_readwrite ("width", &tnc::GridMapView::width)
        .def_readwrite ("height", &tnc::GridMapView::height)
        .def_readwrite ("resolution", &tnc::GridMapView::resolution)
        .def_readwrite ("origin_x", &tnc::GridMapView::origin_x)
        .def_readwrite ("origin_y", &tnc::GridMapView::origin_y)
        .def ("in_bounds", &tnc::GridMapView::inBounds)
        .def ("at", &tnc::GridMapView::at);

}
