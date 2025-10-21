#include "calibration/focal_length_initialization.hpp"

#include <gtest/gtest.h>

using namespace reprojection::calibration;

TEST(CalibrationFocalLengthInitialization, TestFitCircle) {
    // Four points that sit exactly on the circle (x-2)^2 + (y-2)^2 = 1
    Eigen::MatrixX2d const data{{1, 2}, {3, 2}, {2, 1}, {2, 3}};

    auto const [center, radius]{FitCircle(data)};
    EXPECT_TRUE(center.isApproxToConstant(2));
    EXPECT_EQ(radius, 1);
}