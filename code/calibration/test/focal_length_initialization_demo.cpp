#include <gtest/gtest.h>

#include "focal_length_initialization.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;
using namespace reprojection::calibration;

// NOTE(Jack): This demo will count on the fact that the points, ids, pixels are correspondent!

TEST(XXX, YYYY) {
    // NOTE(Jack): We will use this to simulate the mythical non-existent feature frame where one row and one column
    // wrap completely around in a circle! This is not realistic, but allows us to figure out the basic logic.
    Eigen::MatrixX2d const pixels1{{0, 1}, {2, 1}, {1, 0}, {1, 2}};  // (x-1)^2 + (y-1)^2 = 1
    Eigen::MatrixX2d const pixels2{{1, 2}, {3, 2}, {2, 1}, {2, 3}};  // (x-2)^2 + (y-2)^2 = 1



    EXPECT_EQ(1, 2);
}