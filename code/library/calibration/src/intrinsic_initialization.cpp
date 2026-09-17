#include "intrinsic_initialization.hpp"

#include "projection_functions/camera_model.hpp"

namespace reprojection::calibration {

IntrinsicsInitializer SelectInitializationStrategy(CameraModel const camera_model) {
    IntrinsicsInitializer initializer;
    if (camera_model == CameraModel::DoubleSphere) {
        initializer = projection_functions::DoubleSphere::Initialize;
    } else if (camera_model == CameraModel::Eucm) {
        initializer = projection_functions::Eucm::Initialize;
    } else if (camera_model == CameraModel::Pinhole) {
        initializer = projection_functions::Pinhole::Initialize;
    } else if (camera_model == CameraModel::PinholeRadtan4) {
        initializer = projection_functions::PinholeRadtan4::Initialize;
    } else if (camera_model == CameraModel::Ucm) {
        initializer = projection_functions::Ucm::Initialize;
    } else {
        // LCOV_EXCL_START
        throw std::runtime_error(
            "LIBRARY IMPLEMENTATION ERROR - InitializeIntrinsics() 'initializer' logic not implemented for: " +
            ToString(camera_model));
        // LCOV_EXCL_STOP
    }

    return initializer;
}  // LCOV_EXCL_LINE

}  // namespace reprojection::calibration
