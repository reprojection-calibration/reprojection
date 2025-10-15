#include "matrix_utilities.hpp"

#include <gtest/gtest.h>

#include "multiple_view_geometry_data_generator.hpp"

using namespace reprojection_calibration::pnp;

TEST(MatrixUtilities, TestInterleaveRowWise) {
    MvgFrameGenerator const generator{MvgFrameGenerator()};
    MvgFrame const frame_i{generator.Generate()};

    Eigen::MatrixX2d const interleaved_pixels{InterleaveRowWise(frame_i.pixels)};

    EXPECT_EQ(interleaved_pixels.rows(), 100);
    // First pixel is duplicated
    EXPECT_TRUE(interleaved_pixels.row(0).isApprox(frame_i.pixels.row(0)));
    EXPECT_TRUE(interleaved_pixels.row(1).isApprox(frame_i.pixels.row(0)));
    // And for good measure lets check that the second pixel is duplicated too :)
    EXPECT_TRUE(interleaved_pixels.row(2).isApprox(frame_i.pixels.row(1)));
    EXPECT_TRUE(interleaved_pixels.row(3).isApprox(frame_i.pixels.row(1)));
}

TEST(MatrixUtilities, TestNormalizeColumnWise) {
    MvgFrameGenerator const generator{MvgFrameGenerator()};
    MvgFrame const frame_i{generator.Generate()};

    auto const [normalized_test_pixels, tf_pixels]{NormalizeColumnWise(frame_i.pixels)};
    EXPECT_FLOAT_EQ(normalized_test_pixels.rowwise().norm().mean(), std::sqrt(frame_i.pixels.cols()));

    auto const [normalized_test_points, tf_points]{NormalizeColumnWise(frame_i.points)};
    EXPECT_FLOAT_EQ(normalized_test_points.rowwise().norm().mean(), std::sqrt(frame_i.points.cols()));
}
