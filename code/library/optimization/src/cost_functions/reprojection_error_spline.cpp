#include "cost_functions/reprojection_error_spline.hpp"

#include "projection_functions/double_sphere.hpp"
#include "projection_functions/pinhole.hpp"
#include "projection_functions/pinhole_radtan4.hpp"
#include "projection_functions/unified_camera_model.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::optimization::cost_functions {

using namespace projection_functions;

// TODO(Jack): This function is very very similar to the one for the plain old non-spline projection cost function. Are
// we missing the concept that would let us just do this once?
// WARN(Jack): This is an overloaded function, this error will not help the user identify which Create() failed!
ceres::CostFunction* Create(CameraModel const projection_type, ImageBounds const& bounds, Vector2d const& pixel,
                            Vector3d const& point_w, double const u_i, uint64_t const delta_t_ns) {
    if (projection_type == CameraModel::DoubleSphere) {
        return ReprojectionErrorSpline_T<DoubleSphere>::Create(pixel, point_w, bounds, u_i, delta_t_ns);
    } else if (projection_type == CameraModel::Pinhole) {
        return ReprojectionErrorSpline_T<Pinhole>::Create(pixel, point_w, bounds, u_i, delta_t_ns);
    } else if (projection_type == CameraModel::PinholeRadtan4) {
        return ReprojectionErrorSpline_T<PinholeRadtan4>::Create(pixel, point_w, bounds, u_i, delta_t_ns);
    } else if (projection_type == CameraModel::UnifiedCameraModel) {
        return ReprojectionErrorSpline_T<UnifiedCameraModel>::Create(pixel, point_w, bounds, u_i, delta_t_ns);
    } else {
        throw std::runtime_error(
            "The requested camera model is not supported by the reprojection::optimization::Create() function.");  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::optimization::cost_functions
