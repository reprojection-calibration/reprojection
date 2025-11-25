#include "types/calibration_types.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(TypesCalibrationTypes, TestBundleMaskOperator) {
    Bundle const bundle{MatrixX2d{{1, 2}, {3, 4}, {5, 6}, {7, 8}},
                        MatrixX3d{{0, 0, 0}, {7, 8, 9}, {4, 5, 6}, {1, 2, 3}}};

    // All ids requested and present
    Bundle const all{bundle(Array4i{0, 1, 2, 3})};
    EXPECT_TRUE(all.pixels.isApprox(bundle.pixels));
    EXPECT_TRUE(all.points.isApprox(bundle.points));

    // Only second and last id requested and present
    Bundle const sub_bundle{bundle(Array2i{1, 3})};
    EXPECT_EQ(sub_bundle.pixels.rows(), 2);  // Could have also checked points :)
    EXPECT_TRUE(sub_bundle.pixels.row(0).isApprox(bundle.pixels.row(1)));
    EXPECT_TRUE(sub_bundle.pixels.row(1).isApprox(bundle.pixels.row(3)));
    EXPECT_TRUE(sub_bundle.points.row(0).isApprox(bundle.points.row(1)));
    EXPECT_TRUE(sub_bundle.points.row(1).isApprox(bundle.points.row(3)));
}