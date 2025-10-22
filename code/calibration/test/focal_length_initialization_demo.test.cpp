#include "calibration/focal_length_initialization_demo.hpp"

#include <gtest/gtest.h>

using namespace reprojection;
using namespace reprojection::calibration;

// NOTE(Jack): This demo will count on the fact that the points, ids, pixels are correspondent!

TEST(XXX, FFFFF) {
    // NOTE(Jack): We will use this to simulate the mythical non-existent feature frame where one row and one column
    // wrap completely around in a circle! This is not realistic, but allows us to figure out the basic logic.
    // First four pixels are (x-1)^2 + (y-1)^2 = 1 and the last four are // (x-2)^2 + (y-2)^2 = 1
    Eigen::MatrixX2d const pixels{{0, 1}, {2, 1}, {1, 0}, {1, 2}, {1, 2}, {3, 2}, {2, 1}, {2, 3}};
    Eigen::MatrixX3d const points(pixels.rows(), 3);  // Empty as they are not used
    // Because the two sets of pixels are totally unrelated we cannot have them share any rows or columns, therefore the
    // second circle starts at row 1 and is in column 4 - no shared rows or columns at all!
    Eigen::ArrayX2i const indices{{0, 0}, {0, 1}, {0, 2}, {0, 3}, {1, 4}, {2, 4}, {3, 4}, {4, 4}};

    double const f{InitializeFocalLengthFromTarget(pixels, indices)};

    EXPECT_FLOAT_EQ(f, 0.45015815);
}

TEST(XXX, FFFFFCCC) {
    Eigen::ArrayX2i const indices{{0, 0}, {0, 1}, {0, 2}, {0, 3}, {1, 4}, {2, 4}, {3, 4}, {4, 4}};

    Eigen::ArrayXi const row_0_mask{MaskTargetIndicesDimension(indices, 0, Dimension::Row)};
    EXPECT_TRUE(row_0_mask.isApprox(Eigen::Array<int, 4, 1>{0, 1, 2, 3}));

    Eigen::ArrayXi const col_0_mask{MaskTargetIndicesDimension(indices, 4, Dimension::Col)};
    EXPECT_TRUE(col_0_mask.isApprox(Eigen::Array<int, 4, 1>{4, 5, 6, 7}));
}