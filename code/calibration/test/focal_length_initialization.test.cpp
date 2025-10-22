#include "calibration/focal_length_initialization.hpp"

#include <gtest/gtest.h>

#include "projection_functions/double_sphere.hpp"

using namespace reprojection;
using namespace reprojection::calibration;

// COLINEAR WILL ALWAYS FAIL! I.e. the lines through the principal points will always be colinear for models like ds
TEST(CalibrationFocalLengthInitialization, TestXXX) {
    Eigen::Array<double, 6, 1> const intrinsics{600, 600, 360, 240, 0.1, 0.2};

    Eigen::MatrixX3d const horizontal_points{{-360, 100, 600}, {-240, 100, 600}, {-120, 100, 600}, {0, 100, 600},
                                             {120, 100, 600},  {240, 100, 600},  {320, 100, 600}};
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

    auto const circle1{FitCircle(horizontal_pixels)};
    ASSERT_TRUE(circle1.has_value());

    auto const circle2{FitCircle(vertical_pixels)};
    ASSERT_TRUE(circle2.has_value());

    auto const intersection_points{CircleCircleIntersection(circle1.value(), circle2.value())};
    ASSERT_TRUE(intersection_points.has_value());

    auto const [p1, p2]{intersection_points.value()};
    double const f{(p1 - p2).norm() / M_PI};

    // ERROR(Jack): Off by and order of magnitude, where does that come from? Look at intrinsics to see the real value
    EXPECT_EQ(f, 6035.6078019296829);
}

TEST(CalibrationFocalLengthInitialization, TestCircleCircleIntersection) {
    std::tuple<Eigen::Vector2d, double> const c1{{0, 0}, 1};
    std::tuple<Eigen::Vector2d, double> const c2{{2, 0}, 2};

    auto const points{CircleCircleIntersection(c1, c2)};
    ASSERT_TRUE(points.has_value());

    auto const [p1, p2]{points.value()};
    EXPECT_TRUE(p1.isApprox(Eigen::Vector2d{0.25, -0.96824583655185426}));
    EXPECT_TRUE(p2.isApprox(Eigen::Vector2d{0.25, 0.96824583655185426}));
}

TEST(CalibrationFocalLengthInitialization, TestCircleCircleIntersectionSeperate) {
    std::tuple<Eigen::Vector2d, double> const c1{{1, 1}, 1};
    std::tuple<Eigen::Vector2d, double> const c2{{3, 3}, 1};  // Completely outside c1

    EXPECT_EQ(CircleCircleIntersection(c1, c2), std::nullopt);
}

TEST(CalibrationFocalLengthInitialization, TestCircleCircleIntersectionContained) {
    std::tuple<Eigen::Vector2d, double> const c1{{1, 1}, 1};
    std::tuple<Eigen::Vector2d, double> const c2{{1.5, 1.5}, 0.1};  // Completely inside c1

    EXPECT_EQ(CircleCircleIntersection(c1, c2), std::nullopt);
}

TEST(CalibrationFocalLengthInitialization, TestCircleCircleIntersectionCoincident) {
    std::tuple<Eigen::Vector2d, double> const c1{{1, 1}, 1};

    // Coincident - infinite number of solutions
    EXPECT_EQ(CircleCircleIntersection(c1, c1), std::nullopt);
}

TEST(CalibrationFocalLengthInitialization, TestCircleCircleIntersectionOnePoint) {
    std::tuple<Eigen::Vector2d, double> const c1{{1, 1}, 1};
    std::tuple<Eigen::Vector2d, double> const c2{{3, 1}, 1};  // Only one intersection point - no calibration interest

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