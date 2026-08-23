#include "projection_functions/spherical_projections.hpp"

namespace reprojection::projection_functions {

// NOTE(Jack): In "The Double Sphere Camera Model, Usenko Et al. 2018" they do not present the UCM unprojection in their
// "numerically more stable" formulation. Regardless it is implemented here in that formulation following the EUCM
// implementation with beta=1.
std::optional<Array3d> SphericalUnprojection(Eigen::Array<double, 6, 1> const& intrinsics, ImageBounds const& bounds,
                                             Array2d const& pixel) {
    auto const ray{Pinhole::Unproject(intrinsics.head<3>(), bounds, pixel)};
    if (not ray) {
        return std::nullopt;
    }

    double const& mx{ray.value()[0]};
    double const& my{ray.value()[1]};

    double const& alpha{intrinsics[3]};
    double const& xi{intrinsics[4]};
    double const& beta{intrinsics[5]};

    double const r2{mx * mx + my * my};
    bool const valid{alpha <= 0.5 ? true : r2 < 1.0 / (beta * (2 * alpha - 1))};
    if (not valid) {
        return std::nullopt;
    }

    double const mz{(1 - beta * alpha * alpha * r2) /
                    (alpha * std::sqrt(1.0 - (2 * alpha - 1.0) * beta * r2) + 1.0 - alpha)};  // Eqn. 50
    const double mz2{mz * mz};

    const double d{(mz * xi + std::sqrt(mz2 + (1 - xi * xi) * r2)) / (mz2 + r2)};  // Eqn. 46 fraction part
    Array3d m{mx, my, mz};
    m *= d;
    m(2) -= xi;

    return m;
}

}  // namespace reprojection::projection_functions