#include "projection_cost_function.hpp"

#include "projection_functions/double_sphere.hpp"
#include "projection_functions/pinhole.hpp"
#include "projection_functions/pinhole_radtan4.hpp"
#include "projection_functions/unified_camera_model.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

ceres::CostFunction* Create(CameraModel const projection_type, Vector2d const& pixel, Vector3d const& point) {
    if (projection_type == CameraModel::DoubleSphere) {
        return ProjectionCostFunction_T<projection_functions::DoubleSphere>::Create(pixel, point);
    } else if (projection_type == CameraModel::Pinhole) {
        return ProjectionCostFunction_T<projection_functions::Pinhole>::Create(pixel, point);
    } else if (projection_type == CameraModel::PinholeRadtan4) {
        return ProjectionCostFunction_T<projection_functions::PinholeRadtan4>::Create(pixel, point);
    } else if (projection_type == CameraModel::UnifiedCameraModel) {
        return ProjectionCostFunction_T<projection_functions::UnifiedCameraModel>::Create(pixel, point);
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
