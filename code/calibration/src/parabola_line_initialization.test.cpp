#include "parabola_line_initialization.hpp"

#include <gtest/gtest.h>

#include "projection_functions/unified_camera_model.hpp"

using namespace reprojection;

// NOTE(Jack): Reading the original paper "Single View Point Omnidirectional Camera Calibration from Planar Grids" you
// see that there derivation for initializing the focal length (also sometimes referred too as gamma) makes an
// assumption, and that is that the camera/mirror being modeled is a parabola. In the context of the ucm camera model
// this is the case where xi = 1. In this test we test exactly that case, using a ucm camera with xi=1 and we see that
// we get exactly the focal length we expect. Satisfying!!!
TEST(CalibrationParaboleLineInitialization, TestParabolaLineInitialization) {
    Eigen::Array2d const principal_point{360, 240};
    Eigen::Array<double, 5, 1> const intrinsics{600, 600, principal_point[0], principal_point[1], 1};
    MatrixX3d const horizontal_points{{-360, 100, 600}, {-240, 100, 600}, {-120, 100, 600}, {0, 100, 600},
                                      {120, 100, 600},  {240, 100, 600},  {320, 100, 600}};

    // TODO(Jack): Eliminate copy and paste by adding a helper to projection_functions like we already have for pinhole.
    // This loop is copy several times across the testing.
    MatrixX2d horizontal_pixels(horizontal_points.rows(), 2);
    for (int i{0}; i < horizontal_points.rows(); ++i) {
        horizontal_pixels.row(i) =
            projection_functions::UnifiedCameraModel::Project<double>(intrinsics, horizontal_points.row(i));
    }

    auto const f{calibration::ParabolaLineInitialization(principal_point, horizontal_pixels)};
    ASSERT_TRUE(f.has_value());
    EXPECT_FLOAT_EQ(f.value(), 600);
}

TEST(CalibrationParaboleLineInitialization, TestTooFewPixels) {
    Array2d const principal_point{360, 240};
    MatrixX2d const three_pixels{{0, 0}, {100, 100}, {200, 200}};

    auto const f{calibration::ParabolaLineInitialization(principal_point, three_pixels)};
    EXPECT_FALSE(f.has_value());
}

TEST(CalibrationParaboleLineInitialization, GGGG) {
    Array2d const principal_point{360, 240};
    MatrixX2d const three_pixels{{0, 0}, {100, 100}, {200, 200}, {300, 300}};

    auto const f{calibration::ParabolaLineInitialization(principal_point, three_pixels)};
    EXPECT_FALSE(f.has_value());
}
