#include "calibration/initialize_focal_length.hpp"

#include <gtest/gtest.h>

#include "eigen_utilities/grid.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::calibration {}  // namespace reprojection::calibration

using namespace reprojection;

TEST(CalibrationInitializeFocalLength, TestSortIntoRowsAndCols) {
    Bundle const bundle{MatrixX2d{{1, 2}, {3, 4}, {5, 6}, {7, 8}, {0, 0}, {0, 0}},
                        MatrixX3d{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {7, 8, 9}, {4, 5, 6}, {1, 2, 3}}};
    ArrayX2i const indices{eigen_utilities::GenerateGridIndices(2, 3)};
    ExtractedTarget const target{bundle, indices};

    auto const [rows, cols]{calibration::SortIntoRowsAndCols(target)};

    EXPECT_EQ(std::size(rows), 2);
    EXPECT_TRUE(rows[0].pixels.isApprox(bundle.pixels.block(0, 0, 3, 2)));

    EXPECT_EQ(std::size(cols), 3);
    EXPECT_TRUE(cols[0].pixels.isApprox(Eigen::MatrixX2d{{1, 2}, {7, 8}}));
}