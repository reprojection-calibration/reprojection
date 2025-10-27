#include <gtest/gtest.h>

#include "projection_functions/double_sphere.hpp"
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
    double const c3{C(2)};                        // a = gamma * nz --> c3
    double const c4{C(3)};                        // b = nz / gamma --> c4
    double const t{c1 * c1 + c2 * c2 + c3 * c4};  // a * b = gamma * nz * nz / gamma = nz * nz --> c3 * c4

    if (t < 0) {
        // Magnitude of a line's normal vector N has to be positive!
        return std::nullopt;
    }

    double const d{1.0 / std::sqrt(t)};
    double const nx{d * c1};
    double const ny{d * c2};
    double const xxx{nx * nx + ny * ny};  // TODO(Jack): What to name this...?

    if (xxx < 0.95) {
        // Line is radial - threshold 0.95 taken directly from the reference publication
        return std::nullopt;
    }

    double const nz{std::sqrt(1 - xxx)};  // Solve using the constraint 1 = x^2 + y^2 + z^2 for line normal N
    double const gamma{d * c3 / nz};

    return gamma;
}

}  // namespace reprojection::calibration

using namespace reprojection;

TEST(CalibrationParaboleLineInitialization, TestParabolaLineInitialization) {
    Eigen::Array<double, 6, 1> const intrinsics{600, 600, 360, 240, 0.1, 0.2};

    MatrixX3d const horizontal_points{{-360, 100, 600}, {-240, 100, 600}, {-120, 100, 600}, {0, 100, 600},
                                      {120, 100, 600},  {240, 100, 600},  {320, 100, 600}};
    // TODO(Jack): Eliminate copy and paste by adding a helper to projection_functions like we already have for pinhole.
    // This loop is copy and pasted here twice.
    MatrixX2d horizontal_pixels(horizontal_points.rows(), 2);
    for (int i{0}; i < horizontal_points.rows(); ++i) {
        horizontal_pixels.row(i) =
            projection_functions::DoubleSphere::Project<double>(intrinsics, horizontal_points.row(i));
    }

    auto const f{calibration::ParabolaLineInitialization({360, 240}, horizontal_pixels)};

    ASSERT_TRUE(f.has_value());
    EXPECT_FLOAT_EQ(f.value(), 1557.0753);
}