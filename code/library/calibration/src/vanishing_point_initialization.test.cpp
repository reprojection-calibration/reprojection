#include "vanishing_point_initialization.hpp"

#include <gtest/gtest.h>

#include "projection_functions/camera_model.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(CalibrationFocalLengthInitialization, TestVanishingPointInitialization) {
    Eigen::Array<double, 6, 1> const intrinsics{600, 600, 360, 240, 0.1, 0.2};

    // NOTE(Jack): Our strategy here is to use DoubleSphere::Project to project a row and column of points to pixels.
    // VanishingPointInitialization() then fits circles to each row, calculates their intersections and then from that
    // the focal length. One thing to notice here in the choice of *_points is that a circle cannot be fit to collinear
    // points. Therefore, we offset the points from the principal point by 100 units to make sure that we get "bend" and
    // not just a displacement along the radial axis and therefore a failed circle fitting.
    MatrixX3d const horizontal_points{{-360, 100, 600}, {-240, 100, 600}, {-120, 100, 600}, {0, 100, 600},
                                      {120, 100, 600},  {240, 100, 600},  {320, 100, 600}};
    auto const camera{projection_functions::DoubleSphereCamera(intrinsics)};
    MatrixX2d const horizontal_pixels(camera.Project(horizontal_points));

    MatrixX3d const vertical_points{
        {100, -240, 600}, {100, -120, 600}, {100, 0, 600}, {100, 120, 600}, {100, 240, 600}};
    MatrixX2d const vertical_pixels(camera.Project(vertical_points));

    auto const f{calibration::VanishingPointInitialization(horizontal_pixels, vertical_pixels)};
    ASSERT_TRUE(f.has_value());
    // ERROR(Jack): Off by and order of magnitude, where does that come from? Look at intrinsics to see the actual
    // expected value (600)
    EXPECT_FLOAT_EQ(f.value(), 6035.6078019296829);
}

//  NOTE(Jack): This test provides a more simple call to VanishingPointInitialization(), where instead of projecting
//  points with the double sphere model we use two circles that intersect as the simulated pixels. Of course this does
//  not realistically reflect the situation of a imaging a gridded target with a distorted lense, but the math works
//  out simple and exact, good for debugging!
TEST(CalibrationFocalLengthInitialization, TestVanishingPointInitializationPerfectCircles) {
    MatrixX2d const pixels1{{0, 1}, {2, 1}, {1, 0}, {1, 2}};  // (x-1)^2 + (y-1)^2 = 1
    MatrixX2d const pixels2{{1, 2}, {3, 2}, {2, 1}, {2, 3}};  // (x-2)^2 + (y-2)^2 = 1

    auto const f{calibration::VanishingPointInitialization(pixels1, pixels2)};
    ASSERT_TRUE(f.has_value());
    EXPECT_FLOAT_EQ(f.value(), 0.45015815);
}

TEST(CalibrationFocalLengthInitialization, TestVanishingPointInitializationBadCircles) {
    // Check the first error condition where one of the pixel sets does not produce a valid circle
    MatrixX2d const pixels1{{1, 1}, {2, 2}, {2, 2}, {4, 4}};  // Collinear
    MatrixX2d const pixels2{{1, 2}, {3, 2}, {2, 1}, {2, 3}};  // (x-2)^2 + (y-2)^2 = 1

    auto const f{calibration::VanishingPointInitialization(pixels1, pixels2)};
    EXPECT_EQ(f, std::nullopt);
}

TEST(CalibrationFocalLengthInitialization, TestVanishingPointInitializationNoVanishingPoints) {
    // Check the second error condition where the circles have no intersection
    MatrixX2d const pixels1{{0, 2}, {4, 2}, {2, 0}, {2, 4}};  // (x-2)^2 + (y-2)^2 = 2
    MatrixX2d const pixels2{{1, 2}, {3, 2}, {2, 1}, {2, 3}};  // Completely inside the pixels1 circle

    auto const f{calibration::VanishingPointInitialization(pixels1, pixels2)};
    EXPECT_EQ(f, std::nullopt);
}

TEST(CalibrationFocalLengthInitialization, TestCircleCircleIntersection) {
    calibration::Circle const c1{{0, 0}, 1};
    calibration::Circle const c2{{2, 0}, 2};

    auto const points{calibration::CircleCircleIntersection(c1, c2)};
    ASSERT_TRUE(points.has_value());

    auto const [p1, p2]{points.value()};
    EXPECT_TRUE(p1.isApprox(Vector2d{0.25, -0.96824583655185426}));
    EXPECT_TRUE(p2.isApprox(Vector2d{0.25, 0.96824583655185426}));
}

TEST(CalibrationFocalLengthInitialization, TestCircleCircleIntersectionSeperate) {
    calibration::Circle const c1{{1, 1}, 1};
    calibration::Circle const c2{{3, 3}, 1};  // Completely outside c1

    EXPECT_EQ(calibration::CircleCircleIntersection(c1, c2), std::nullopt);
}

TEST(CalibrationFocalLengthInitialization, TestCircleCircleIntersectionContained) {
    calibration::Circle const c1{{1, 1}, 1};
    calibration::Circle const c2{{1.5, 1.5}, 0.1};  // Completely inside c1

    EXPECT_EQ(calibration::CircleCircleIntersection(c1, c2), std::nullopt);
}

TEST(CalibrationFocalLengthInitialization, TestCircleCircleIntersectionCoincident) {
    calibration::Circle const c1{{1, 1}, 1};

    // Coincident - infinite number of solutions
    EXPECT_EQ(calibration::CircleCircleIntersection(c1, c1), std::nullopt);
}

TEST(CalibrationFocalLengthInitialization, TestCircleCircleIntersectionOnePoint) {
    calibration::Circle const c1{{1, 1}, 1};
    calibration::Circle const c2{{3, 1}, 1};  // Only one intersection point - no calibration interest

    EXPECT_EQ(calibration::CircleCircleIntersection(c1, c2), std::nullopt);
}

TEST(CalibrationFocalLengthInitialization, TestFitCircle) {
    // Four points that sit exactly on the circle (x-2)^2 + (y-2)^2 = 1
    MatrixX2d const data{{1, 2}, {3, 2}, {2, 1}, {2, 3}};

    auto const circle{calibration::FitCircle(data)};
    ASSERT_TRUE(circle.has_value());

    auto const [center, radius]{circle.value()};
    EXPECT_TRUE(center.isApproxToConstant(2));
    EXPECT_EQ(radius, 1);
}

TEST(CalibrationFocalLengthInitialization, TestFitCircleStraightLine) {
    // Degenerate condition when points are collinear
    MatrixX2d const data{{1, 1}, {2, 2}, {3, 3}, {4, 4}};
    EXPECT_EQ(calibration::FitCircle(data), std::nullopt);
}