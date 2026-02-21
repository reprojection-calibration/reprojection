#include "sparse_utilities.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;

// {stride, count, ground truth matrix}
std::vector<std::tuple<int, int, MatrixXd>> test_cases{// Kitty-corner
                                                       {2, 2,
                                                        Matrix4d{{1, 1, 0, 0},  //
                                                                 {1, 1, 0, 0},
                                                                 {0, 0, 1, 1},
                                                                 {0, 0, 1, 1}}},
                                                       // Total overlap - overlapped elements accumulate
                                                       {0, 10, Matrix2d{{10, 10}, {10, 10}}},
                                                       // Partial overlap
                                                       {1, 3,
                                                        Matrix4d{{1, 1, 0, 0},  //
                                                                 {1, 2, 1, 0},
                                                                 {0, 1, 2, 1},
                                                                 {0, 0, 1, 1}}},
                                                       // Disjoint
                                                       {3, 2,
                                                        Eigen::Matrix<double, 5, 5>{{1, 1, 0, 0, 0},  //
                                                                                    {1, 1, 0, 0, 0},
                                                                                    {0, 0, 0, 0, 0},
                                                                                    {0, 0, 0, 1, 1},
                                                                                    {0, 0, 0, 1, 1}}},
                                                       // Empty - should it not actually return a 0x0 matrix?
                                                       {0, 0,
                                                        Matrix2d{{0, 0},  //
                                                                 {0, 0}}}};

TEST(SplineSparseUtilities, TestDiagonalSparseMatrix) {
    Matrix2d const block{{1, 1},  //
                         {1, 1}};

    for (auto const& [stride, count, gt_mat] : test_cases) {
        auto const mat{spline::DiagonalSparseMatrix(block, stride, count)};

        EXPECT_TRUE(mat.isApprox(gt_mat)) << "Result:\n" << mat << "\nexpected result:\n" << gt_mat;
    }
}
