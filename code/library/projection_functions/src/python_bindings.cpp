#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "projection_functions/camera_model.hpp"
#include "projection_functions/pinhole.hpp"

namespace py = pybind11;

namespace reprojection::projection_functions {

void BindImageBounds(py::module_& module) {
    py::class_<ImageBounds>(module, "ImageBounds")
        .def(py::init<double, double, double, double>(), py::arg("u_min"), py::arg("u_max"), py::arg("v_min"),
             py::arg("v_max"))
        .def_readwrite("u_min", &ImageBounds::u_min)
        .def_readwrite("u_max", &ImageBounds::u_max)
        .def_readwrite("v_min", &ImageBounds::v_min)
        .def_readwrite("v_max", &ImageBounds::v_max);
}

template <typename T_Model>
void BindCamera(py::module_& module, std::string const& name) {
    // A concrete camera is an instantiation of the Camera pure virtual interface class - remember this class was added
    // for convenience and not the templated ceres optimization!
    py::class_<Camera_T<T_Model>, Camera>(module, name.c_str())
        .def(py::init<Eigen::Array<double, T_Model::Size, 1> const&, ImageBounds const&>(), py::arg("intrinsics"),
             py::arg("bounds"));
}

void BindCameras(py::module_& module) {
    py::class_<Camera>(module, "Camera").def("Project", &Camera::Project).def("Unproject", &Camera::Unproject);

    BindCamera<DoubleSphere>(module, "DoubleSphereCamera");
    BindCamera<Eucm>(module, "EucmCamera");
    BindCamera<Pinhole>(module, "PinholeCamera");
    BindCamera<PinholeRadtan4>(module, "PinholeRadtan4Camera");
    BindCamera<Ucm>(module, "UcmCamera");
}

}  // namespace reprojection::projection_functions

PYBIND11_MODULE(projection_function_python_binding, module) {
    reprojection::projection_functions::BindImageBounds(module);
    reprojection::projection_functions::BindCameras(module);
}