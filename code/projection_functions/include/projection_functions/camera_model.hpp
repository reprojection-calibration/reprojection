#pragma once

#include "projection_functions/pinhole.hpp"
#include "projection_functions/pinhole_radtan4.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

// High level design explanation:
//
// There is the everyday need to have the ability to project/unproject points and pixels. We need a generic interface
// that lets us do that. The code here are the tools used to achieve that generic interface. Part one of the toolset is
// the Camera class, that provides the interface definition using two pure virtual functions (Project and Unproject)
// that capture essence of what we define a "camera" to be. The second part of the toolset is the CameraT class, which
// provides a templated override of the Camera class, giving us the ability to concrete implementations without code
// duplication, from the existing projection function classes (projection_functions/pinhole.hpp,double_sphere.hpp etc.)
// Those classes, which are static classes, were originally designed to serve primarily as namespaces, but also now
// allow us to easily use the CameraT class to create specific cameras that operate on eigen arrays without a ton of
// code duplication (see the use of "TModel::template ...").
//
// Something that is unsettled at this point (28.10.2025), and I want to delay the decision on as long as possible, is
// how we will actually use the projection functions in the optimization. I think we want to avoid using a pure virtual
// base class there because then we will have vtable calls and it will be "slower" (I did not benchmark this, that is
// just my imagination talking!) than using templated code directly. This is still an open point, and is the explanation
// for why I did not just go all in using the Camera base class everywhere. In some places it is great because it keeps
// templates away from the code, but we need to look at possible problems later when we get to the optimization.
//
// A different topic, but an idea which should also be written down here in a central location: The Project functions
// are templated and the Unproject functions are not, because the former will be used in the nonlinear optimization and
// need to handle ceres::Jet types. The Unproject functions will (I think!) not be required to handle anything besides a
// double type because they are used only for tasks not directly related to the nonlinear optimization. I had started
// out templating both the Project and Unproject functions because it looked consistent, but when developing the Camera
// base class this turned into a problem. The side effect of this is that the Project implementation in Camera_T uses
// "T_Model::template Project<double>()" but the Unproject method uses only "T_Model::Unproject()".

class Camera {
   public:
    virtual ~Camera() = default;

    virtual MatrixX2d Project(MatrixX3d const& points_co) const = 0;

    virtual MatrixX3d Unproject(MatrixX2d const& pixels) const = 0;
};

template <typename T_Model, typename T_Intrinsics>
class Camera_T : public Camera {
   public:
    explicit Camera_T(T_Intrinsics const& intrinsics) : intrinsics_{intrinsics} {}

    MatrixX2d Project(MatrixX3d const& points_co) const override {
        Eigen::MatrixX2d pixels(points_co.rows(), 2);
        for (int i{0}; i < points_co.rows(); ++i) {
            pixels.row(i) = T_Model::template Project<double>(intrinsics_, points_co.row(i));
        }

        return pixels;
    }  // LCOV_EXCL_LINE

    MatrixX3d Unproject(MatrixX2d const& pixels) const override {
        Eigen::MatrixX3d rays_co(pixels.rows(), 3);
        for (int i{0}; i < pixels.rows(); ++i) {
            rays_co.row(i) = T_Model::Unproject(intrinsics_, pixels.row(i));
        }

        return rays_co;
    }  // LCOV_EXCL_LINE

   private:
    T_Intrinsics intrinsics_;
};

using PinholeCamera = Camera_T<Pinhole, Array4d>;
using PinholeRadtan4Camera = Camera_T<PinholeRadtan4, Eigen::Array<double, 8, 1>>;

}  // namespace reprojection::projection_functions
