#include "sparse_utilities.hpp"

namespace reprojection::spline {

Eigen::SparseMatrix<double> DiagonalSparseMatrix(MatrixXd const& block, size_t const stride, size_t const count) {
    if (not(block.rows() == block.cols())) {
        throw std::runtime_error("Only accepts square blocks");
    }

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(block.size() * count);

    for (size_t n{0}; n < count; n++) {
        size_t const start_index{n * stride};

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
