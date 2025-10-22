
#include <gtest/gtest.h>

#include "focal_length_initialization.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;
using namespace reprojection::calibration;

// NOTE(Jack): This demo will count on the fact that the points, ids, pixels are correspondent!

TEST(XXX, FFFFF) {
    // NOTE(Jack): We will use this to simulate the mythical non-existent feature frame where one row and one column
    // wrap completely around in a circle! This is not realistic, but allows us to figure out the basic logic.
    // First four pixels are (x-1)^2 + (y-1)^2 = 1 and the last four are // (x-2)^2 + (y-2)^2 = 1
    Eigen::MatrixX2d const pixels{{0, 1}, {2, 1}, {1, 0}, {1, 2}, {1, 2}, {3, 2}, {2, 1}, {2, 3}};
    Eigen::MatrixX3d const points(pixels.rows(), 3);  // Empty as they are not used
    // Normally the {0,0} ID will be shared but not here because of the two circles are totally separate and not
    // coming from a calibration target.
    Eigen::ArrayX2i const indices{{0, 0}, {0, 1}, {0, 2}, {0, 3}, {1, 0}, {2, 0}, {3, 0}, {4, 0}};

    types::FeatureFrame const frame{pixels, points, indices};

    EXPECT_EQ(1, 2);
}