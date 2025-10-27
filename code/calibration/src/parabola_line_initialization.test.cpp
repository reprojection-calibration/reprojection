#include "parabola_line_initialization.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "projection_functions/unified_camera_model.hpp"

using namespace reprojection;

// NOTE(Jack): Reading the original paper "Single View Point Omnidirectional Camera Calibration from Planar Grids" you
// see that there derivation for initializing the focal length (also sometimes referred to as gamma) makes an
// assumption, and that is that the camera/mirror being modeled is a parabola. In the context of the ucm camera model
// this is the case where xi = 1. In our testing that uses a ucm camera with xi=1 we see that we get exactly the
// focal length we expect. Satisfying!!!

TEST(CalibrationParabolaLineInitialization, TestParabolaLineInitialization) {
    Eigen::Array2d const principal_point{360, 240};
    Eigen::Array<double, 5, 1> const intrinsics{600, 600, principal_point[0], principal_point[1], 1};

    Eigen::ParametrizedLine<double, 3> const line({-100, -100, 600}, {60, 0, 20});
    MatrixX3d points_co(10, 3);
    for (int i{0}; i < 10; i++) {
        points_co.row(i) = line.pointAt(i);
    }


    MatrixX2d linear_pixels(points_co.rows(), 2);
    for (int i{0}; i < points_co.rows(); ++i) {
        linear_pixels.row(i) = projection_functions::UnifiedCameraModel::Project<double>(intrinsics, points_co.row(i));
    }

    std::cout << linear_pixels << std::endl;

    auto const f{calibration::ParabolaLineInitialization(principal_point, linear_pixels)};
    ASSERT_TRUE(f.has_value());
    EXPECT_FLOAT_EQ(f.value(), 600);
}
