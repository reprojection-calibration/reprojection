#include "matrix_utilities.hpp"

#include <gtest/gtest.h>

#include "testing_mocks/mvg_generator.hpp"

using namespace reprojection;
using namespace reprojection::pnp;

TEST(PnpMatrixUtilities, TestInterleaveRowWise) {
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator()};
    testing_mocks::MvgFrame const frame{generator.Generate(0.5)};  // Arbitrary spot in the middle

    Eigen::MatrixX2d const interleaved_pixels{InterleaveRowWise(frame.pixels)};

    EXPECT_EQ(interleaved_pixels.rows(), 50);
    // First pixel is duplicated
    EXPECT_TRUE(interleaved_pixels.row(0).isApprox(frame.pixels.row(0)));
    EXPECT_TRUE(interleaved_pixels.row(1).isApprox(frame.pixels.row(0)));
    // And for good measure lets check that the second pixel is duplicated too :)
    EXPECT_TRUE(interleaved_pixels.row(2).isApprox(frame.pixels.row(1)));
    EXPECT_TRUE(interleaved_pixels.row(3).isApprox(frame.pixels.row(1)));
}

TEST(PnpMatrixUtilities, TestNormalizeColumnWise) {
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator()};
    testing_mocks::MvgFrame const frame{generator.Generate(0.5)};  // Arbitrary spot in the middle

    auto const [normalized_test_pixels, tf_pixels]{NormalizeColumnWise(frame.pixels)};
    EXPECT_FLOAT_EQ(normalized_test_pixels.rowwise().norm().mean(), std::sqrt(frame.pixels.cols()));

    auto const [normalized_test_points, tf_points]{NormalizeColumnWise(frame.points)};
    EXPECT_FLOAT_EQ(normalized_test_points.rowwise().norm().mean(), std::sqrt(frame.points.cols()));
}
