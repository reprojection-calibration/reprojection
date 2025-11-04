#include "projection_functions/unified_camera_model.hpp"

namespace reprojection::projection_functions {

// TODO(Jack): A continuation of my theory that UCM and DS are practically the same exact same! Could be wrong... see
// the ucm projection function for more context.
Array3d UnifiedCameraModel::Unproject(Eigen::Array<double, Size, 1> const& intrinsics, Array2d const& pixel) {
    double const alpha{0};
    Array6d const ds_intrinsics(intrinsics(0), intrinsics(1), intrinsics(2), intrinsics(3), intrinsics(4), alpha);

    return DoubleSphere::Unproject(ds_intrinsics, pixel);
}

}  // namespace reprojection::projection_functions
