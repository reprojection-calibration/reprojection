#include "projection_functions/pinhole.hpp"

#include "types/eigen_types.hpp"

namespace reprojection::projection_functions {

Array3d Pinhole::Unproject(Eigen::Array<double, Size, 1> const& intrinsics, Array2d const& pixel) {
    double const& fx{intrinsics[0]};
    double const& fy{intrinsics[1]};
    double const& cx{intrinsics[2]};
    double const& cy{intrinsics[3]};

    double const& u{pixel[0]};
    double const& v{pixel[1]};

    double const x_cam{(u - cx) / fx};
    double const y_cam{(v - cy) / fy};

    // NOTE(Jack): Setting the z value here to 1 essentially captures the pure essence of the loss of
    // information/scale/depth associated with the projective transform. This returns a "ray" or direction, not a 3D
    // point in the camera optical frame.
    return {x_cam, y_cam, 1};
}

}  // namespace reprojection::projection_functions
