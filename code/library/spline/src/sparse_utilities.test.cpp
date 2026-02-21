#include <gtest/gtest.h>

#include <Eigen/SparseCore>

#include "types/eigen_types.hpp"

namespace reprojection::spline {

// See section "Filling a sparse matrix" - https://libeigen.gitlab.io/eigen/docs-nightly/group__TutorialSparse.html
Eigen::SparseMatrix<double> DiagonalSparseMatrix(MatrixXd const& block, int const stride, size_t const count) {
    if (not(block.rows() == block.cols())) {
        throw std::runtime_error("Only accepts square blocks");
    }

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(block.size() * count);

    for (size_t n{0}; n < count; n++) {
        int const start_index{static_cast<int>(n * stride)};

        for (Eigen::Index row{0}; row < block.rows(); row++) {
            for (Eigen::Index col{0}; col < block.cols(); col++) {
                triplets.push_back(
                    {static_cast<int>(start_index + row), static_cast<int>(start_index + col), block(row, col)});
            }
        }
    }

    size_t const size{(block.rows() * count) + (stride - block.rows()) * (count - 1)};
    Eigen::SparseMatrix<double> mat(size, size);
    mat.setFromTriplets(std::cbegin(triplets), std::cend(triplets));

    return mat;
}

}  // namespace reprojection::spline

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