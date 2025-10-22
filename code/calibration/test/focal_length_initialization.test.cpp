#include "calibration/focal_length_initialization.hpp"

#include <gtest/gtest.h>

using namespace reprojection::calibration;

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