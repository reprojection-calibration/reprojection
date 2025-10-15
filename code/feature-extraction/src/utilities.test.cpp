#include "utilities.hpp"

#include <gtest/gtest.h>

using namespace reprojection_calibration::feature_extraction;

TEST(Utilities, TestGenerateGridIndices) {
    int const rows{3};
    int const cols{4};

    Eigen::ArrayX2i const grid_indices{GenerateGridIndices(rows, cols)};

    EXPECT_EQ(grid_indices.rows(), rows * cols);
    // Heuristically check the first grid row that it is ((0,0), (0,1), (0,2), (0,3))
    EXPECT_TRUE(grid_indices.col(0).topRows(cols).isApprox(Eigen::ArrayXi::Zero(cols)));
    EXPECT_TRUE(grid_indices.col(1).topRows(cols).isApprox(Eigen::ArrayXi::LinSpaced(cols, 0, cols)));
}

TEST(Utilities, TestMaskIndices) {
    Eigen::Array<int, 5, 1> const grid_indices{1, 0, 1, 0, 1};

    Eigen::ArrayXi const mask_indices{MaskIndices(grid_indices)};

    EXPECT_EQ(mask_indices.rows(), 3);
    EXPECT_TRUE(mask_indices.isApprox(Eigen::Array<int, 3, 1>{0, 2, 4}));
}

TEST(Utilities, TestAlternatingSum) {
    double const sum_1{AlternatingSum(2, 0.5, 0.2)};
    EXPECT_EQ(sum_1, 0.5 + 0.2);

    double const sum_2{AlternatingSum(3, 0.5, 0.2)};
    EXPECT_EQ(sum_2, 0.5 + 0.2 + 0.5);

    double const sum_null{AlternatingSum(0, 0.5, 0.2)};
    EXPECT_EQ(sum_null, 0);
}