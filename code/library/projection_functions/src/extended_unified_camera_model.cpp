#include "projection_functions/extended_unified_camera_model.hpp"

namespace reprojection::projection_functions {

// TODO IMPLEMENT!!!!
std::optional<Array3d> ExtendedUnifiedCameraModel::Unproject(Eigen::Array<double, Size, 1> const& intrinsics,
                                                             ImageBounds const& bounds, Array2d const& pixel) {
    (void)intrinsics;
    (void)bounds;
    (void)pixel;

    return std::nullopt;
}

}  // namespace reprojection::projection_functions