#include "calibration/focal_length_initialization.hpp"

#include <gtest/gtest.h>

using namespace reprojection::calibration;

// Adopted from https://stackoverflow.com/questions/3349125/circle-circle-intersection-points which copy and pasted from
// here https://paulbourke.net/geometry/circlesphere/
// TODO(Jack): Do float math better and more expressive in the last condition - here and everywhere!
std::optional<std::tuple<Eigen::Vector2d, Eigen::Vector2d>> CircleCircleIntersection(
    std::tuple<Eigen::Vector2d, double> const& c1, std::tuple<Eigen::Vector2d, double> const& c2) {
    auto const [P0, r0]{c1};
    auto const [P1, r1]{c2};

    double const d{(P1 - P0).norm()};
    if (d > r0 + r1) {
        return std::nullopt;  // Do not overlap at all
    } else if (d < std::abs(r0 - r1)) {
        return std::nullopt;  // One is inside the other
    } else if (d < 1e-8 and std::abs(r0 - r1) < 1e-8) {
        return std::nullopt;  // Coincident circles - infinite number of solutions
    }

    double const a{(r0 * r0 - r1 * r1 + d * d) / (2 * d)};
    double const h{std::sqrt(r0 * r0 - a * a)};
    if (h < 1e-8) {
        return std::nullopt;  // Circles intersect at one point, maybe generally interesting but not for us calibrating
    }

    Eigen::Vector2d const P2{P0 + a * (P1 - P0) / d};

    // TODO(Jack): Can we eloquently eliminate the copy and paste here? It is all done just to swap the signs for the
    // points, but maybe this is ok here.
    Eigen::Vector2d const P3_i{P2(0) + h * (P1(1) - P0(1)) / d, P2(1) - h * (P1(0) - P0(0)) / d};
    Eigen::Vector2d const P3_j{P2(0) - h * (P1(1) - P0(1)) / d, P2(1) + h * (P1(0) - P0(0)) / d};

    return std::tuple<Eigen::Vector2d, Eigen::Vector2d>{P3_i, P3_j};
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

    auto const [center, radius]{FitCircle(data)};
    EXPECT_TRUE(center.isApproxToConstant(2));
    EXPECT_EQ(radius, 1);
}