#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

namespace reprojection::calibration {

// principal_point is provided in pixel coordinates and pixels is at least four pixels from from four 3D features which
// lie on a straight line (i.e. the row/column of a calibration pattern.)
std::optional<double> ParabolaLineInitialization(Vector2d const& principal_point, MatrixX2d const& pixels) {
    MatrixX2d const pixels_c{pixels.rowwise() - principal_point.transpose()};

    Eigen::MatrixX4d P(pixels.rows(), 4);
    P.col(0) = pixels_c.col(0);
    P.col(1) = pixels_c.col(1);
    P.col(2).setConstant(0.5);
    P.col(3) = -pixels_c.rowwise().squaredNorm() / 2.0;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    svd.compute(P, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::Vector4d const C{svd.matrixV().col(3)};

    // We are not normalized so we do not yet refer to that variables (ex. c1 or c2) as part of the line normal N
    double const c1{C(0)};
    double const c2{C(1)};
    double const a{C(2)};                       // a = gamma * nz --> c3
    double const b{C(3)};                       // b = nz / gamma --> c4
    double const t{c1 * c1 + c2 * c2 + a * b};  // a * b = gamma * nz * nz / gamma --> nz * nz

    if (t < 0) {
        // Magnitude of a line's normal vector N has to be positive!
        return std::nullopt;
    }

    double const nx{c1 / std::sqrt(t)};
    double const ny{c1 / std::sqrt(t)};
    double const xxx{nx * nx + ny * ny};

    if (xxx < 0.95) {
        // Line is radial - threshold 0.95 taken directly from the reference publication
        return std::nullopt;
    }

    double const c3{a / std::sqrt(t)};
    double const nz{std::sqrt(1 - xxx)};  // Solve using the constraint 1 = x^2 + y^2 + z^2 for N
    double const gamma{c3 / nz};

    return gamma;
}

}  // namespace reprojection::calibration

using namespace reprojection;

TEST(CalibrationParaboleLineInitialization, TestParabolaLineInitialization) {
    // Linear line that does not pass through the principal point (i.e. non-radial)
    MatrixX2d const pixels{{100, 100}, {200, 150}, {300, 200}, {400, 250}};

    std::cout << "Output" << std::endl;
    std::cout << calibration::ParabolaLineInitialization({320, 240}, pixels).value() << std::endl;
}