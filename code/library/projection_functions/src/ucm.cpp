#include "projection_functions/ucm.hpp"

namespace reprojection::projection_functions {

std::optional<Array3d> Ucm::Unproject(Eigen::Array<double, Size, 1> const& intrinsics, ImageBounds const& bounds,
                                      Array2d const& pixel) {
    double const xi{0};
    double const beta{1};
    Eigen::Array<double, 6, 1> const spherical_intrinsics(intrinsics(0), intrinsics(1), intrinsics(2), intrinsics(3),
                                                          xi, beta);

    return SphericalUnprojection(spherical_intrinsics, bounds, pixel);
}

}  // namespace reprojection::projection_functions