#include "utilities.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(CalibrationUtilities, TestSortIntoRowsAndCols) {
    ExtractedTarget target;
    target.bundle.pixels = ArrayX2d::Random(6, 2);
    target.bundle.points = Eigen::ArrayX3d::Random(6, 3);
    target.indices = Eigen::ArrayX2i{{0, 0},  //
                                     {0, 1},  //
                                     {0, 2},  //
                                     {1, 0},  //
                                     {1, 1},  //
                                     {1, 2}};

    auto const [rows, cols]{calibration::SortIntoRowsAndCols(target)};
    EXPECT_EQ(std::size(rows), 2);
    EXPECT_EQ(std::size(cols), 3);

    // We could also have checked the point values here, but pixels is also fine
    EXPECT_TRUE(rows[0].pixels.isApprox(target.bundle.pixels.topRows<3>()));
    EXPECT_TRUE(rows[1].pixels.isApprox(target.bundle.pixels.bottomRows<3>()));

    // Because the columns are interleaved we need to test the columns manually, so we only check the first one.
    EXPECT_TRUE(cols[0].pixels.row(0).isApprox(target.bundle.pixels.row(0)));
    EXPECT_TRUE(cols[0].pixels.row(1).isApprox(target.bundle.pixels.row(3)));
}