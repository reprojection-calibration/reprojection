#pragma once

#include <ceres/autodiff_cost_function.h>

#include "ceres_geometry.hpp"
#include "projection_functions/projection_class_concept.hpp"
#include "types/camera_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

/**
 * \brief Generates a camera model specific projection cost function for use in the optimization.
 *
 * This function delineates (or I hope it does) between templated and non-templated code for the camera projection
 * functions. Ceres provides us the `ceres::CostFunction` abstraction, and we take advantage of that here to constrain
 * the knowledge that the optimizer has to have about the camera model being optimized. Essentially the optimizer
 * problem itself does not care at all if it is optimizing a pinhole or double sphere camera model, because all it needs
 * to know is that it gets a ceres cost function.
 *
 * Because the ceres cost function abstraction and how we integrate it here in this function directly with our camera
 * model projection functions, we can keep all consuming code free of templates and pure virtual classes! Cool :) Note
 * that if the pointer is not assigned to a ceres problem then you need to call delete yourself, otherwise the memory
 * will leak!
 */
ceres::CostFunction* Create(CameraModel const projection_type, Vector2d const& pixel, Vector3d const& point);

// NOTE(Jack): Relation between eigen and ceres: https://groups.google.com/g/ceres-solver/c/7ZH21XX6HWU
// WARN(Jack): As we move past simple mono camera calibration we might find out that it is not the best option and that
// the pose/transform will play a less central role here and instead be part of the spline. This is unclear at this
// point, so I am gonna hold off from adding any documentation, it is not so hard to understand anyway.
template <typename T_Model>
// WARN(Jack): Technically for the cost function we only need the HasIntrinsicsSize<> and CanProject<> concepts.
// Therefore, using the ProjectionClass<> concept is overkill because it requires template parameters to also satisfy
// the CanUnproject<> requirement. That being said, in the context of the entire project, when some introduces a new
// camera model they are not gonna just want to use it (or I do not want them) only here with ProjectionCostFunction_T.
// Therefore, here and all other places where we expect a camera model we require compliance with the full
// ProjectionClass<> concept. This makes the project more homogenous and will catch errors more quickly. That being said
// maybe this also indicates we have an incorrect abstraction because information which is not strictly needed (i.e.
// that CanUnproject<> is required) is present here. Let's see how this plays out in the long term!
    requires projection_functions::ProjectionClass<T_Model>
class ProjectionCostFunction_T {
   public:
    ProjectionCostFunction_T(Vector2d const& pixel, Vector3d const& point) : pixel_{pixel}, point_{point} {}

    template <typename T>
    bool operator()(T const* const intrinsics_ptr, T const* const pose_ptr, T* const residual) const {
        Eigen::Map<Eigen::Vector<T, 6> const> pose(pose_ptr);
        Eigen::Vector<T, 3> const point_co{TransformPoint<T>(pose, point_.cast<T>())};

        Eigen::Map<Eigen::Array<T, T_Model::Size, 1> const> intrinsics(intrinsics_ptr);
        auto const pixel{T_Model::template Project<T>(intrinsics, point_co)};

        if (pixel.has_value()) {
            Array2<T> const& _pixel{pixel.value()};

            residual[0] = T(pixel_[0]) - _pixel[0];
            residual[1] = T(pixel_[1]) - _pixel[1];

            return true;
        } else {
            return false;
        }
    }

    static ceres::CostFunction* Create(Vector2d const& pixel, Vector3d const& point) {
        return new ceres::AutoDiffCostFunction<ProjectionCostFunction_T, 2, T_Model::Size, 6>(
            new ProjectionCostFunction_T(pixel, point));
    }

    Vector2d pixel_;
    Vector3d point_;
};

}  // namespace  reprojection::optimization
