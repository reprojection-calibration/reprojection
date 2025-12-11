#include "parabola_line_initialization.hpp"

#include <Eigen/SVD>

namespace reprojection::calibration {

// NOTEJack): In the original paper there is an assertion/check that t > 0. This comes from the fact that t represent a
// normal vector and therefore its square magnitude has to be positive. I think the intent for the check is that if the
// SVD fails is some special way, due to the relation of c3 and c4, we might get a negative t value. That being said I
// am unable to get a scenario in a test that produces a negative t value. Let us keep our eyes peeled here to see if
// this becomes a problem in the initialization! My experience so far shows that all error conditions are actually
// caught with the nxnx_nyny > 0.95 "non-radial" line check.
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
    double const& c1{C(0)};
    double const& c2{C(1)};
    double const& c3{C(2)};                       // a = gamma * nz --> c3
    double const& c4{C(3)};                       // b = nz / gamma --> c4
    double const t{c1 * c1 + c2 * c2 + c3 * c4};  // a * b = gamma * nz * nz / gamma = nz * nz --> c3 * c4

    // WARN(Jack): In the reference paper there is a check here to make sure that t is positive! See note above. If it
    // turns out we do get negative t here then the square root might cause problems...
    double const d{1.0 / std::sqrt(t)};
    double const nx{d * c1};
    double const ny{d * c2};
    double const nxnx_nyny{nx * nx + ny * ny};

    // NOTE(Jack): This seems to be a pretty aggressive threshold. In some cases a line within 100 pixels of the origin
    // can be classified as "radial", that feels like too much. Testing with real data over time will tell if this
    // matters or not.
    double const nznz{1 - nxnx_nyny};  // Using the constraint 1 = x^2 + y^2 + z^2 for line normal N
    if (nznz < 0.05) {
        // Too close to image center (i.e. line is "radial" which is a singularity case)
        return std::nullopt;
    }
    double const gamma{d * c3 / std::sqrt(nznz)};

    return gamma;
}

}  // namespace reprojection::calibration
