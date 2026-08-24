#include "projection_functions/eucm.hpp"

namespace reprojection::projection_functions {

std::optional<Array3d> Eucm::Unproject(Eigen::Array<double, Size, 1> const& intrinsics, ImageBounds const& bounds,
                                       Array2d const& pixel) {
    double const xi{0};
    Eigen::Array<double, 6, 1> const spherical_intrinsics(intrinsics(0), intrinsics(1), intrinsics(2), intrinsics(3),
                                                          xi, intrinsics(4));

    return SphericalUnprojection(spherical_intrinsics, bounds, pixel);
}

}  // namespace reprojection::projection_functions