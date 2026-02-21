#include <gtest/gtest.h>

#include <Eigen/SparseCore>

#include "types/eigen_types.hpp"

namespace reprojection::spline {

// See section "Filling a sparse matrix" - https://libeigen.gitlab.io/eigen/docs-nightly/group__TutorialSparse.html
std::vector<Eigen::Triplet<double>> DiagonalSparseTriplets(MatrixXd const& block, int const step, size_t const count) {
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(block.size() * count);

    for (size_t n{0}; n < count; n++) {
        int const start_row{static_cast<int>(n * step)};
        int const start_col{start_row};

        for (Eigen::Index row{0}; row < block.rows(); row++) {
            for (Eigen::Index col{0}; col < block.cols(); col++) {
                triplets.push_back(
                    {static_cast<int>(start_row + row), static_cast<int>(start_col + col), block(row, col)});
            }
        }
    }

    return triplets;
}

}  // namespace reprojection::spline

using namespace reprojection;

TEST(SplineSparseUtilities, TestDiagonalSparseTriplets) {
    Matrix2d const block{{1, 1},  //
                         {1, 1}};

    auto const triplets{spline::DiagonalSparseTriplets(block, 2, 2)};
    Eigen::SparseMatrix<double> mat(4, 4);
    mat.setFromTriplets(std::cbegin(triplets), std::cend(triplets));

    Matrix4d const gt_mat{{1, 1, 0, 0},  //
                          {1, 1, 0, 0},
                          {0, 0, 1, 1},
                          {0, 0, 1, 1}};

    EXPECT_TRUE(mat.isApprox(gt_mat, 1e-6)) << "Result:\n" << mat << "\nexpected result:\n" << gt_mat;
}

TEST(SplineSparseUtilities, TestDiagonalSparseTriplets2) {
    Matrix2d const block{{1, 1},  //
                         {1, 1}};

    auto const triplets{spline::DiagonalSparseTriplets(block, 1, 3)};
    Eigen::SparseMatrix<double> mat(4, 4);
    mat.setFromTriplets(std::cbegin(triplets), std::cend(triplets));

    Matrix4d const gt_mat{{1, 1, 0, 0},  //
                          {1, 2, 1, 0},
                          {0, 1, 2, 1},
                          {0, 0, 1, 1}};

    EXPECT_TRUE(mat.isApprox(gt_mat, 1e-6)) << "Result:\n" << mat << "\nexpected result:\n" << gt_mat;
}

TEST(SplineSparseUtilities, TestDiagonalSparseTriplets3) {
    Matrix2d const block{{1, 1},  //
                         {1, 1}};

    auto const triplets{spline::DiagonalSparseTriplets(block, 3, 2)};
    Eigen::SparseMatrix<double> mat(5, 5);
    mat.setFromTriplets(std::cbegin(triplets), std::cend(triplets));

    Eigen::Matrix<double, 5, 5> const gt_mat{{1, 1, 0, 0, 0},  //
                                             {1, 1, 0, 0, 0},
                                             {0, 0, 0, 0, 0},
                                             {0, 0, 0, 1, 1},
                                             {0, 0, 0, 1, 1}};

    EXPECT_TRUE(mat.isApprox(gt_mat, 1e-6)) << "Result:\n" << mat << "\nexpected result:\n" << gt_mat;
}