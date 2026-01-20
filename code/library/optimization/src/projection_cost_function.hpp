#pragma once

#include <ceres/autodiff_cost_function.h>

#include "ceres_geometry.hpp"
#include "projection_functions/projection_class_concept.hpp"
#include "types/calibration_types.hpp"
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
ceres::CostFunction* Create(CameraModel const projection_type, ImageBounds const& bounds, Vector2d const& pixel,
                            Vector3d const& point);

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
    ProjectionCostFunction_T(Vector2d const& pixel, Vector3d const& point, ImageBounds const& bounds)
        : pixel_{pixel}, point_{point}, bounds_{bounds} {}

    template <typename T>
    bool operator()(T const* const intrinsics_ptr, T const* const pose_ptr, T* const residual) const {
        Eigen::Map<Eigen::Vector<T, 6> const> pose(pose_ptr);
        Vector3<T> const point_co{TransformPoint<T>(pose, point_.cast<T>())};

        Eigen::Map<Eigen::Array<T, T_Model::Size, 1> const> intrinsics(intrinsics_ptr);
        auto const pixel{T_Model::template Project<T>(intrinsics, bounds_, point_co)};

        if (pixel.has_value()) {
            Array2<T> const& _pixel{pixel.value()};

            residual[0] = T(pixel_[0]) - _pixel[0];
            residual[1] = T(pixel_[1]) - _pixel[1];

            return true;
        } else {
            // NOTE(Jack): TLDR is - instead of leaving the residual empty and returning false to signal cost
            // function evaluation failure we fill it with a large constant value and return true.
            //
            // I could write an entire essay here, but let me try to keep it short and simple. I was under the belief
            // (and still am as of 20.01.2026) that when cost function evaluation fails we should return false. However,
            // when you return false it has dramatic implications for solving. There are two primary cases, (1) when you
            // assemble a problem and the cost function given the initial parameters returns false and (2) during
            // optimization the cost function starts to return false due to the newly optimized parameter values.
            //
            // The first problem is an immediate dealbreaker and ceres will not even let you start to optimize that
            // problem - which makes sense. If the solver cannot ever evaluate the cost function how is it supposed to
            // know in which direction to take a step to start looking for the optimized parameters?
            //
            // The second problem manifests itself during the optimization itself after it has start. If during the
            // optimization any single cost function evaluates to false, ceres screams and says "Treating it as a step
            // with infinite cost" killing that iteration, and then in the next step reducing the trust region radius
            // and looking for parameters in another direction.
            //
            // Ok, so now with this understanding you say: "well jack the first problem we need to prevent during
            // problem construction of course, but the second problem the optimizer can solve itself." And maybe you are
            // right, or maybe you can find the problem I could not, but the answer is not as obvious as you hope.
            //
            // I tried for a while to simply return false here and let the optimizer use that as a signal to discover
            // which parts of the parameter space are bad for business. For whatever reason this did not work and the
            // "Treating it as a step with infinite cost" was constantly printed during optimization and the end results
            // as viewed in the dashboard were not satisfactory. You can see others discuss the idea here
            // https://groups.google.com/g/ceres-solver/c/PPrZo0rxCno/m/kH98OIw6CgAJ?pli=1
            //
            // In the end the best solution I settled on was to set the residual to a constant arbitrary non-zero value
            // when the projection evaluation fails. This tells the optimizer that hey these are bad points and they do
            // not provide us any information for solving, but just because I have some bad points does not mean I need
            // to kill the entire problem evaluation.
            // NOTE(Jack): The value of the residual here should be larger than the expected final residual given a
            // successful optimization. For example for reprojection error I would expect that when it is successful the
            // error is under one pixel. So setting the residuals to 10 here clearly signals this is a failure
            // condition.
            residual[0] = T(10);
            residual[1] = T(10);

            return true;
        }
    }

    static ceres::CostFunction* Create(Vector2d const& pixel, Vector3d const& point, ImageBounds const& bounds) {
        return new ceres::AutoDiffCostFunction<ProjectionCostFunction_T, 2, T_Model::Size, 6>(
            new ProjectionCostFunction_T(pixel, point, bounds));
    }

    Vector2d pixel_;
    Vector3d point_;
    ImageBounds bounds_;
};

}  // namespace  reprojection::optimization
