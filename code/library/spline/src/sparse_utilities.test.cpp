#include "sparse_utilities.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(SplineSparseUtilities, TestDiagonalSparseMatrix) {
    Matrix2d const block{{1, 1},  //
                         {1, 1}};

    auto const mat{spline::DiagonalSparseMatrix(block, 2, 2)};

    Matrix4d const gt_mat{{1, 1, 0, 0},  //
                          {1, 1, 0, 0},
                          {0, 0, 1, 1},
                          {0, 0, 1, 1}};

    EXPECT_TRUE(mat.isApprox(gt_mat, 1e-6)) << "Result:\n" << mat << "\nexpected result:\n" << gt_mat;
}

TEST(SplineSparseUtilities, TestDiagonalSparseMatrix2) {
    Matrix2d const block{{1, 1},  //
                         {1, 1}};

    auto const mat{spline::DiagonalSparseMatrix(block, 1, 3)};

    Matrix4d const gt_mat{{1, 1, 0, 0},  //
                          {1, 2, 1, 0},
                          {0, 1, 2, 1},
                          {0, 0, 1, 1}};

    EXPECT_TRUE(mat.isApprox(gt_mat, 1e-6)) << "Result:\n" << mat << "\nexpected result:\n" << gt_mat;
}

TEST(SplineSparseUtilities, TestDiagonalSparseMatrix3) {
    Matrix2d const block{{1, 1},  //
                         {1, 1}};

    auto const mat{spline::DiagonalSparseMatrix(block, 3, 2)};

    Eigen::Matrix<double, 5, 5> const gt_mat{{1, 1, 0, 0, 0},  //
                                             {1, 1, 0, 0, 0},
                                             {0, 0, 0, 0, 0},
                                             {0, 0, 0, 1, 1},
                                             {0, 0, 0, 1, 1}};

    EXPECT_TRUE(mat.isApprox(gt_mat, 1e-6)) << "Result:\n" << mat << "\nexpected result:\n" << gt_mat;
}