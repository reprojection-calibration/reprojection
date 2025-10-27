#include "parabola_line_initialization.hpp"

#include <gtest/gtest.h>

#include "projection_functions/unified_camera_model.hpp"

using namespace reprojection;

TEST(CalibrationParaboleLineInitialization, TestParabolaLineInitialization) {
    Eigen::Array<double, 5, 1> const intrinsics{600, 600, 360, 240, 1};

    MatrixX3d const horizontal_points{{-360, 100, 600}, {-240, 100, 600}, {-120, 100, 600}, {0, 100, 600},
                                      {120, 100, 600},  {240, 100, 600},  {320, 100, 600}};
    // TODO(Jack): Eliminate copy and paste by adding a helper to projection_functions like we already have for pinhole.
    // This loop is copy and pasted here twice.
    MatrixX2d horizontal_pixels(horizontal_points.rows(), 2);
    for (int i{0}; i < horizontal_points.rows(); ++i) {
        horizontal_pixels.row(i) =
            projection_functions::UnifiedCameraModel::Project<double>(intrinsics, horizontal_points.row(i));
    }

    auto const f{calibration::ParabolaLineInitialization({360, 240}, horizontal_pixels)};

    ASSERT_TRUE(f.has_value());
    EXPECT_FLOAT_EQ(f.value(), 600);
}