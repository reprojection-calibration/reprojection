#include "matrix_utilities.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(PnpMatrixUtilities, TestInterleaveRowWise) {
    MatrixX2d const a{{0, 0}, {1, 1}, {2, 2}};
    Eigen::MatrixX2d const interleaved_pixels{pnp::InterleaveRowWise(a)};

    EXPECT_EQ(interleaved_pixels.rows(), 2 * a.rows());
    // First row is duplicated
    EXPECT_TRUE(interleaved_pixels.row(0).isApprox(a.row(0)));
    EXPECT_TRUE(interleaved_pixels.row(1).isApprox(a.row(0)));
    // And for good measure lets check that the second row is duplicated too :)
    EXPECT_TRUE(interleaved_pixels.row(2).isApprox(a.row(1)));
    EXPECT_TRUE(interleaved_pixels.row(3).isApprox(a.row(1)));
}

TEST(PnpMatrixUtilities, TestNormalizeColumnWise) {
    MatrixX2d const a{{0, 0}, {1, 1}, {2, 2}};
    auto const [normalized_test_pixels, _]{pnp::NormalizeColumnWise(a)};
    EXPECT_FLOAT_EQ(normalized_test_pixels.rowwise().norm().mean(), std::sqrt(a.cols()));

    MatrixX3d const b{{0, 0, 0}, {1, 1, 1}, {2, 2, 2}};
    auto const [normalized_test_points, _1]{pnp::NormalizeColumnWise(b)};
    EXPECT_FLOAT_EQ(normalized_test_points.rowwise().norm().mean(), std::sqrt(b.cols()));
}
