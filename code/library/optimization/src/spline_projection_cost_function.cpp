#include "spline_projection_cost_function.hpp"

#include "projection_functions/double_sphere.hpp"
#include "projection_functions/pinhole.hpp"
#include "projection_functions/pinhole_radtan4.hpp"
#include "projection_functions/unified_camera_model.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::optimization {

using namespace projection_functions;

ceres::CostFunction* Create(CameraModel const projection_type, ImageBounds const& bounds, Vector2d const& pixel,
                            Vector3d const& point, double const u_i, uint64_t const delta_t_ns) {
    if (projection_type == CameraModel::DoubleSphere) {
        return SplineProjectionCostFunction_T<DoubleSphere>::Create(pixel, point, bounds, u_i, delta_t_ns);
    } else if (projection_type == CameraModel::Pinhole) {
        return SplineProjectionCostFunction_T<Pinhole>::Create(pixel, point, bounds, u_i, delta_t_ns);
    } else if (projection_type == CameraModel::PinholeRadtan4) {
        return SplineProjectionCostFunction_T<PinholeRadtan4>::Create(pixel, point, bounds, u_i, delta_t_ns);
    } else if (projection_type == CameraModel::UnifiedCameraModel) {
        return SplineProjectionCostFunction_T<UnifiedCameraModel>::Create(pixel, point, bounds, u_i, delta_t_ns);
    } else {
        // NOTE(Jack): The only way we could cover this with a test is to have a member that is part of the CameraModel
        // enum that is not covered here in the conditional. That makes no sense. Therefore, we will supress the
        // code coverage requirement for this line branch of the statement. Furthermore, this is more of an error case
        // rather than an algorithm edge case, therefore not having this covered is not so dangerous.
        throw std::runtime_error(
            "The requested camera model is not supported by the reprojection::optimization::Create() function.");  // LCOV_EXCL_LINE
    }
}

}  // namespace  reprojection::optimization
