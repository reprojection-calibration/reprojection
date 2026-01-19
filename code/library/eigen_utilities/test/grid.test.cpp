#include "eigen_utilities/grid.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(EigenUtiltiesGrid, TestGenerateGridIndices) {
    int const rows{3};
    int const cols{4};

    ArrayX2i const grid_indices{eigen_utilities::GenerateGridIndices(rows, cols)};

    EXPECT_EQ(grid_indices.rows(), rows * cols);
    // Heuristically check the first grid row that it is ((0,0), (0,1), (0,2), (0,3))
    EXPECT_TRUE(grid_indices.col(0).topRows(cols).isApprox(ArrayXi::Zero(cols)));
    EXPECT_TRUE(grid_indices.col(1).topRows(cols).isApprox(ArrayXi::LinSpaced(cols, 0, cols)));
}

TEST(EigenUtiltiesGrid, TestMaskToRowId) {
    Array5b const mask{true, false, true, false, true};

    ArrayXi const mask_indices{eigen_utilities::MaskToRowId(mask)};

    EXPECT_EQ(mask_indices.rows(), 3);
    EXPECT_TRUE(mask_indices.isApprox(Eigen::Array<int, 3, 1>{0, 2, 4}));
}