#include "parabola_line_initialization.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "projection_functions/camera_model.hpp"

using namespace reprojection;

// NOTE(Jack): Reading the original paper "Single View Point Omnidirectional Camera Calibration from Planar Grids" you
// see that there derivation for initializing the focal length (also sometimes referred to as gamma) makes an
// assumption, and that is that the camera/mirror being modeled is a parabola. In the context of the ucm camera model
// this is the case where xi = 1. In our testing that uses a ucm camera with xi=1 we see that we get exactly the
// focal length we expect. Satisfying!!!

// NOTE(Jack): Linear in this sense means that the 3D points that the pixel projections are generated from a collinear
// set of points, the pixels themselves having undergone a ucm projection will not be collinear (unless of course they
// are radial along the x or y-axis :))
std::tuple<MatrixX2d, Vector2d> LinearTestPixels(Vector3d const& origin, Vector3d const& direction) {
    // Generate four points on a line using the provided origin and direction
    Eigen::ParametrizedLine<double, 3> const line(origin, direction);
    MatrixX3d points_co(4, 3);
    for (int i{0}; i < points_co.rows(); i++) {
        points_co.row(i) = line.pointAt(i);
    }

    // Project the four points to pixels using the ucm camera model with xi=1 (i.e. parabola case) and a focal
    // length/gamma of 600
    Array2d const principal_point{360, 240};
    Array5d const intrinsics{600, 600, principal_point[0], principal_point[1], 1};
    auto const camera{projection_functions::UcmCamera(intrinsics)};
    MatrixX2d const pixels(camera.Project(points_co));

    return {pixels, principal_point};
}

TEST(CalibrationParabolaLineInitialization, TestParabolaLineInitialization) {
    auto [pixels, principal_point]{LinearTestPixels({150, 150, 600}, {-10, 5, 0})};
    auto f{calibration::ParabolaLineInitialization(principal_point, pixels)};
    ASSERT_TRUE(f.has_value());
    EXPECT_FLOAT_EQ(f.value(), 600);

    std::tie(pixels, principal_point) = LinearTestPixels({-150, -150, 600}, {5, -10, -10});
    f = calibration::ParabolaLineInitialization(principal_point, pixels);
    ASSERT_TRUE(f.has_value());
    EXPECT_FLOAT_EQ(f.value(), 600);

    std::tie(pixels, principal_point) = LinearTestPixels({150, -150, 600}, {-5, -10, -10});
    f = calibration::ParabolaLineInitialization(principal_point, pixels);
    ASSERT_TRUE(f.has_value());
    EXPECT_FLOAT_EQ(f.value(), 600);

    std::tie(pixels, principal_point) = LinearTestPixels({-150, 150, 600}, {-5, -10, 10});
    f = calibration::ParabolaLineInitialization(principal_point, pixels);
    ASSERT_TRUE(f.has_value());
    EXPECT_FLOAT_EQ(f.value(), 600);
}

TEST(CalibrationParabolaLineInitialization, TestRadialLines) {
    // Passes through origin
    auto [pixels, principal_point]{LinearTestPixels({0, 0, 600}, {10, 10, 0})};
    auto f{calibration::ParabolaLineInitialization(principal_point, pixels)};
    EXPECT_FALSE(f.has_value());

    // Passes through origin
    std::tie(pixels, principal_point) = LinearTestPixels({100, -100, 600}, {-10, 10, 0});
    f = calibration::ParabolaLineInitialization(principal_point, pixels);
    EXPECT_FALSE(f.has_value());

    // Passes near origin
    std::tie(pixels, principal_point) = LinearTestPixels({-25, 25, 600}, {10, 10, 0});
    f = calibration::ParabolaLineInitialization(principal_point, pixels);
    EXPECT_FALSE(f.has_value());
}

TEST(CalibrationParabolaLineInitialization, TestOtherPossibleErrors) {
    // All points at one single location
    auto [pixels, principal_point]{LinearTestPixels({100, 100, 600}, {0, 0, 0})};
    auto f{calibration::ParabolaLineInitialization(principal_point, pixels)};
    EXPECT_FALSE(f.has_value());
}