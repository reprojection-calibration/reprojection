#include "eigen_utilities/grid.hpp"

#include <gtest/gtest.h>

using namespace reprojection::eigen_utilities;

TEST(EigenUtiltiesGrid, TestGenerateGridIndices) {
    int const rows{3};
    int const cols{4};

    Eigen::ArrayX2i const grid_indices{GenerateGridIndices(rows, cols)};

    EXPECT_EQ(grid_indices.rows(), rows * cols);
    // Heuristically check the first grid row that it is ((0,0), (0,1), (0,2), (0,3))
    EXPECT_TRUE(grid_indices.col(0).topRows(cols).isApprox(Eigen::ArrayXi::Zero(cols)));
    EXPECT_TRUE(grid_indices.col(1).topRows(cols).isApprox(Eigen::ArrayXi::LinSpaced(cols, 0, cols)));
}

TEST(EigenUtiltiesGrid, TestMaskIndices) {
    Eigen::Array<int, 5, 1> const indices{1, 0, 1, 0, 1};

    Eigen::ArrayXi const mask_indices{MaskIndices(indices)};

    EXPECT_EQ(mask_indices.rows(), 3);
    EXPECT_TRUE(mask_indices.isApprox(Eigen::Array<int, 3, 1>{0, 2, 4}));
}