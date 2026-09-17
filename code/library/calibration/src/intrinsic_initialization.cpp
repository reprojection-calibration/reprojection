#include "intrinsic_initialization.hpp"

#include "camera_pose_init.hpp"
#include "parabola_line_initialization.hpp"
#include "utilities.hpp"
#include "vanishing_point_initialization.hpp"

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
        throw std::runtime_error(  // LCOV_EXCL_LINE
            "LIBRARY IMPLEMENTATION ERROR - InitializeIntrinsics() 'initializer' logic not implemented for: " +  // LCOV_EXCL_LINE
            ToString(camera_model));  // LCOV_EXCL_LINE
    }

    return initializer;
}

}  // namespace reprojection::calibration
