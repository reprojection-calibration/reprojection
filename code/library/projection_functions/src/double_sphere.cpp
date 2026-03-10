#include "projection_functions/double_sphere.hpp"

#include "projection_functions/pinhole.hpp"

namespace reprojection::projection_functions {

std::optional<Array3d> DoubleSphere::Unproject(Eigen::Array<double, Size, 1> const& intrinsics,
                                               ImageBounds const& bounds, Array2d const& pixel) {
    auto const ray{Pinhole::Unproject(intrinsics.head<4>(), bounds, pixel)};
    if (not ray) {
        return std::nullopt;  // LCOV_EXCL_LINE
    }

    double const& mx{ray.value()[0]};
    double const& my{ray.value()[1]};
    double const r2{mx * mx + my * my};

    double const& alpha{intrinsics[5]};

    bool const valid{alpha <= 0.5 ? true : r2 < 1.0 / (2 * alpha - 1)};
    if (not valid) {
        return std::nullopt;
    }

    double const mz{(1 - alpha * alpha * r2) / (alpha * std::sqrt(1 - (2 * alpha - 1.0) * r2) + 1 - alpha)};  // Eqn. 50

    double const& xi{intrinsics[4]};
    const double mz2{mz * mz};
    // TODO(Jack): What to name this thing?
    const double xxx{(mz * xi + std::sqrt(mz2 + (1 - xi * xi) * r2)) / (mz2 + r2)};  // Eqn. 46 fraction part

    // Execute the rest of equation 46
    Array3d m{mx, my, mz};
    m *= xxx;
    m(2) -= xi;

    return m;
}

}  // namespace reprojection::projection_functions
