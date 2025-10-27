#include "parabola_line_initialization.hpp"

#include <iostream>  // REMOVE

namespace reprojection::calibration {

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
    double const c3{C(2)};                        // a = gamma * nz --> c3
    double const c4{C(3)};                        // b = nz / gamma --> c4
    double const t{c1 * c1 + c2 * c2 + c3 * c4};  // a * b = gamma * nz * nz / gamma = nz * nz --> c3 * c4

    double const d{1.0 / std::sqrt(t)};
    double const nx{d * c1};
    double const ny{d * c2};
    double const nxnx_nyny{nx * nx + ny * ny};

    double const nz{std::sqrt(1 - nxnx_nyny)};  // Solve using the constraint 1 = x^2 + y^2 + z^2 for line normal N
    double const gamma{d * c3 / nz};

    return gamma;
}

}  // namespace reprojection::calibration
