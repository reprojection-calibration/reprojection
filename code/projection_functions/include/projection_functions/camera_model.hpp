#pragma once

#include "projection_functions/double_sphere.hpp"
#include "projection_functions/pinhole.hpp"
#include "projection_functions/pinhole_radtan4.hpp"
#include "projection_functions/projection_class_concept.hpp"
#include "projection_functions/unified_camera_model.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

// TODO(Jack): Move the below comment to any documentation on the projection function base class.
// A different topic, but an idea which should also be written down here in a central location: The Project functions
// are templated and the Unproject functions are not, because the former will be used in the nonlinear optimization and
// need to handle ceres::Jet types. The Unproject functions will (I think!) not be required to handle anything besides a
// double type because they are used only for tasks not directly related to the nonlinear optimization. I had started
// out templating both the Project and Unproject functions because it looked consistent, but when developing the Camera
// base class this turned into a problem. The side effect of this is that the Project implementation in Camera_T uses
// "T_Model::template Project<double>()" but the Unproject method uses only "T_Model::Unproject()".

/**
 * \brief Defines the camera interface (i.e. it is a pure virtual "base" class) for use in non-optimization related
 * code.
 *
 * We often need to project and unproject points/pixels for optimization adjacent tasks, for example generating test
 * data (testing_mocks::MvgGenerator::Project()). This base class allows the consuming code to be
 * agnostic to the specific camera model (ex. pinhole, double sphere, etc.) that is used.
 *
 * In essence this class prevents templated code from spreading throughout the code base, and provides a simple generic
 * interface to apply Project() and Unproject() methods to many data points at one time, regardless of which specific
 * camera projection functions the consumer wants to use. In this context we hard-code the double type (because we do
 * not need the ceres::Jet type required by ceres::AutoDiffCostFunction) and process entire data arrays (ex. n-pixels or
 * n-points) instead of elementwise like we do in the core optimization code.
 */
class Camera {
   public:
    virtual ~Camera() = default;

    /**
     * \brief Defines the camera projection interface. Consumes a set of points in the camera optical frame ("*_co) and
     * returns a set of image space pixels.
     */
    virtual MatrixX2d Project(MatrixX3d const& points_co) const = 0;

    /**
     * \brief Defines the camera unprojection interface. Consumes a set of image space pixels and returns a set of 3D
     * rays in the camera optical frame. Does NOT return the original 3D points (that would require depth information).
     */
    virtual MatrixX3d Unproject(MatrixX2d const& pixels) const = 0;
};

/**
 * \brief Generates the code to implement concrete types from the Camera interface definition class.
 *
 * Given that we have a standard set of projection function classes that implement the project and unproject functions
 * of common camera models (ex. Pinhole or DoubleSphere), there is no reason for us to copy and paste for each camera
 * the looping logic required to apply those functions to arrays. Therefore, we implement that looping logic one time
 * here and instantiate the template once for each projection function class.
 *
 * This class also uses the ::Size attribute of the provided projection function class to parameterize the size of the
 * intrinsics array which varies for each camera model.
 *
 * @tparam T_Model A camera model projection function class that has ::Project<T>(), ::Unproject() and ::Size (ex.
 * Pinhole or DoubleSphere).
 */
template <typename T_Model>
    requires ProjectionClass<T_Model>
class Camera_T : public Camera {
   public:
    explicit Camera_T(Eigen::Array<double, T_Model::Size, 1> const& intrinsics) : intrinsics_{intrinsics} {}

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
    Eigen::Array<double, T_Model::Size, 1> intrinsics_;
};

using DoubleSphereCamera = Camera_T<DoubleSphere>;
using PinholeCamera = Camera_T<Pinhole>;
using PinholeRadtan4Camera = Camera_T<PinholeRadtan4>;
using UcmCamera = Camera_T<UnifiedCameraModel>;

}  // namespace reprojection::projection_functions
