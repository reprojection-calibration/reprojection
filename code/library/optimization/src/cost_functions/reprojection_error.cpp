#include "cost_functions/reprojection_error.hpp"
#include "projection_functions/double_sphere.hpp"
#include "projection_functions/pinhole.hpp"
#include "projection_functions/pinhole_radtan4.hpp"
#include "projection_functions/unified_camera_model.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::optimization::cost_functions {

ceres::CostFunction* Create(CameraModel const projection_type, ImageBounds const& bounds, Vector2d const& pixel,
                            Vector3d const& point) {
    if (projection_type == CameraModel::DoubleSphere) {
        return ReprojectionError_T<projection_functions::DoubleSphere>::Create(pixel, point, bounds);
    } else if (projection_type == CameraModel::Pinhole) {
        return ReprojectionError_T<projection_functions::Pinhole>::Create(pixel, point, bounds);
    } else if (projection_type == CameraModel::PinholeRadtan4) {
        return ReprojectionError_T<projection_functions::PinholeRadtan4>::Create(pixel, point, bounds);
    } else if (projection_type == CameraModel::UnifiedCameraModel) {
        return ReprojectionError_T<projection_functions::UnifiedCameraModel>::Create(pixel, point, bounds);
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - ReprojectionError_T - Create()");  // LCOV_EXCL_LINE
    }
}

}  // namespace  reprojection::optimization
