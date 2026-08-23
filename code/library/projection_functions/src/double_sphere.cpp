#include "projection_functions/double_sphere.hpp"

#include "projection_functions/spherical_projections.hpp"

namespace reprojection::projection_functions {

std::optional<Array3d> DoubleSphere::Unproject(Eigen::Array<double, Size, 1> const& intrinsics,
                                               ImageBounds const& bounds, Array2d const& pixel) {
    double const beta{1};
    Eigen::Array<double, 6, 1> const spherical_projection_intrinsics(intrinsics(0), intrinsics(1), intrinsics(2),
                                                                     intrinsics(4), intrinsics(3), beta);

    return SphericalUnprojection(spherical_projection_intrinsics, bounds, pixel);
}

}  // namespace reprojection::projection_functions
