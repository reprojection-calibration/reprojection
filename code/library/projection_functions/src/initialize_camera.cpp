#include "projection_functions/initialize_camera.hpp"

#include "projection_functions/camera_model.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::projection_functions {

std::unique_ptr<Camera> InitializeCamera(CameraModel const model, ArrayXd const& intrinsics,
                                         ImageBounds const& bounds) {
    if (model == CameraModel::DoubleSphere) {
        return MakeCamera<DoubleSphere>(intrinsics, bounds);
    } else if (model == CameraModel::Pinhole) {
        return MakeCamera<Pinhole>(intrinsics, bounds);
    } else if (model == CameraModel::PinholeRadtan4) {
        return MakeCamera<PinholeRadtan4>(intrinsics, bounds);
    } else if (model == CameraModel::UnifiedCameraModel) {
        return MakeCamera<UnifiedCameraModel>(intrinsics, bounds);
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - invalid camera model");  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::projection_functions