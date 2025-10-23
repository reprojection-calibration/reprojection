#include "focal_length_initialization.hpp"

#include <gtest/gtest.h>

#include "projection_functions/double_sphere.hpp"

using namespace reprojection;
using namespace reprojection::calibration;

TEST(CalibrationFocalLengthInitialization, TestEstimateFocalLength) {
    Eigen::Array<double, 6, 1> const intrinsics{600, 600, 360, 240, 0.1, 0.2};

    // NOTE(Jack): Our strategy here is to use DoubleSphereProjection to project a row and column of points to pixels.
    // EstimateFocalLength() then fits circles to each row, calculates their intersections and then from that the focal
    // length. One thing to notice here in the choice of *_points is that a circle cannot be fit to collinear points.
    // Therefore, we offset the points from the principal point by 100 units to make sure that we get "bend" and not
    // just a displacement along the radial axis and failed circle fitting.
    Eigen::MatrixX3d const horizontal_points{{-360, 100, 600}, {-240, 100, 600}, {-120, 100, 600}, {0, 100, 600},
                                             {120, 100, 600},  {240, 100, 600},  {320, 100, 600}};
    // TODO(Jack): Eliminate copy and paste by adding a helper to projection_functions like we already have for pinhole.
    // This loop is copy and pasted here twice.
    Eigen::MatrixX2d horizontal_pixels(horizontal_points.rows(), 2);
    for (int i{0}; i < horizontal_points.rows(); ++i) {
        horizontal_pixels.row(i) =
            projection_functions::DoubleSphereProjection<double>(intrinsics.data(), horizontal_points.row(i));
    }

    Eigen::MatrixX3d const vertical_points{
        {100, -240, 600}, {100, -120, 600}, {100, 0, 600}, {100, 120, 600}, {100, 240, 600}};
    Eigen::MatrixX2d vertical_pixels(vertical_points.rows(), 2);
    for (int i{0}; i < vertical_points.rows(); ++i) {
        vertical_pixels.row(i) =
            projection_functions::DoubleSphereProjection<double>(intrinsics.data(), vertical_points.row(i));
    }

    auto const f{EstimateFocalLength(horizontal_pixels, vertical_pixels)};
    ASSERT_TRUE(f.has_value());
    // ERROR(Jack): Off by and order of magnitude, where does that come from? Look at intrinsics to see the actual
    // expected value (600)
    EXPECT_FLOAT_EQ(f.value(), 6035.6078019296829);
}

//  NOTE(Jack): This test provides a more simple call to EstimateFocalLength(), where instead of projecting points with
//  the double sphere model we use to circles that intersect as the simulated pixels. Of course this does not
//  realistically reflect the situation of a imaging a gridded target, but the maths works out simply and exactly like
//  we want.
TEST(CalibrationFocalLengthInitialization, TestEstimateFocalLengthPerfectCircles) {
    Eigen::MatrixX2d const pixels1{{0, 1}, {2, 1}, {1, 0}, {1, 2}};  // (x-1)^2 + (y-1)^2 = 1
    Eigen::MatrixX2d const pixels2{{1, 2}, {3, 2}, {2, 1}, {2, 3}};  // (x-2)^2 + (y-2)^2 = 1

    auto const f{EstimateFocalLength(pixels1, pixels2)};
    ASSERT_TRUE(f.has_value());
    EXPECT_FLOAT_EQ(f.value(), 0.45015815);
}

TEST(CalibrationFocalLengthInitialization, TestEstimateFocalLengthBadCircles) {
    // Check the first error condition where one of the pixel sets does not produce a valid circle
    Eigen::MatrixX2d const pixels1{{1, 1}, {2, 2}, {2, 2}, {4, 4}};  // Collinear
    Eigen::MatrixX2d const pixels2{{1, 2}, {3, 2}, {2, 1}, {2, 3}};  // (x-2)^2 + (y-2)^2 = 1

    auto const f{EstimateFocalLength(pixels1, pixels2)};
    EXPECT_EQ(f, std::nullopt);
}

TEST(CalibrationFocalLengthInitialization, TestEstimateFocalLengthNoVanishingPoints) {
    // Check the second error condition where the circles have no intersection
    Eigen::MatrixX2d const pixels1{{0, 2}, {4, 2}, {2, 0}, {2, 4}};  // (x-2)^2 + (y-2)^2 = 2
    Eigen::MatrixX2d const pixels2{{1, 2}, {3, 2}, {2, 1}, {2, 3}};  // Completely inside the pixels1 circle

    auto const f{EstimateFocalLength(pixels1, pixels2)};
    EXPECT_EQ(f, std::nullopt);
}

TEST(CalibrationFocalLengthInitialization, TestCircleCircleIntersection) {
    Circle const c1{{0, 0}, 1};
    Circle const c2{{2, 0}, 2};

    auto const points{CircleCircleIntersection(c1, c2)};
    ASSERT_TRUE(points.has_value());

    auto const [p1, p2]{points.value()};
    EXPECT_TRUE(p1.isApprox(Eigen::Vector2d{0.25, -0.96824583655185426}));
    EXPECT_TRUE(p2.isApprox(Eigen::Vector2d{0.25, 0.96824583655185426}));
}

TEST(CalibrationFocalLengthInitialization, TestCircleCircleIntersectionSeperate) {
    Circle const c1{{1, 1}, 1};
    Circle const c2{{3, 3}, 1};  // Completely outside c1

    EXPECT_EQ(CircleCircleIntersection(c1, c2), std::nullopt);
}

TEST(CalibrationFocalLengthInitialization, TestCircleCircleIntersectionContained) {
    Circle const c1{{1, 1}, 1};
    Circle const c2{{1.5, 1.5}, 0.1};  // Completely inside c1

    EXPECT_EQ(CircleCircleIntersection(c1, c2), std::nullopt);
}

TEST(CalibrationFocalLengthInitialization, TestCircleCircleIntersectionCoincident) {
    Circle const c1{{1, 1}, 1};

    // Coincident - infinite number of solutions
    EXPECT_EQ(CircleCircleIntersection(c1, c1), std::nullopt);
}

TEST(CalibrationFocalLengthInitialization, TestCircleCircleIntersectionOnePoint) {
    Circle const c1{{1, 1}, 1};
    Circle const c2{{3, 1}, 1};  // Only one intersection point - no calibration interest

    EXPECT_EQ(CircleCircleIntersection(c1, c2), std::nullopt);
}

TEST(CalibrationFocalLengthInitialization, TestFitCircle) {
    // Four points that sit exactly on the circle (x-2)^2 + (y-2)^2 = 1
    Eigen::MatrixX2d const data{{1, 2}, {3, 2}, {2, 1}, {2, 3}};

    auto const circle{FitCircle(data)};
    ASSERT_TRUE(circle.has_value());

    auto const [center, radius]{circle.value()};
    EXPECT_TRUE(center.isApproxToConstant(2));
    EXPECT_EQ(radius, 1);
}

TEST(CalibrationFocalLengthInitialization, TestFitCircleStraightLine) {
    // Degenerate condition when points are colinear
    Eigen::MatrixX2d const data{{1, 1}, {2, 2}, {3, 3}, {4, 4}};
    EXPECT_EQ(FitCircle(data), std::nullopt);
}