#pragma once

#include <Eigen/Core>
#include <iostream>
#include <optional>

#include "matrix_utilities.hpp"

namespace reprojection::pnp {

// NOTE(Jack): Constructs the constraint matrix for the dlt problem given two sets of points, it is templated to handle
// both the plane to plane (Dlt22) and plane to points (Dlt23) case. In the Dlt22 case N=3 and A is 2nx9. In the Dlt23
// case N=4 and A is 2nx12.
// The math behind it for the Dlt23 case can be found in "Multiple View Geometry in computer vision", particularly be
// noting that this function stacks Eq. 7.2 to form the A matrix.
template <int N>
Eigen::Matrix<double, Eigen::Dynamic, 3 * N> ConstructA(MatrixX2d const& pixels,
                                                        Eigen::Matrix<double, Eigen::Dynamic, N - 1> const& points) {
    Eigen::Matrix<double, Eigen::Dynamic, 3 * N> A(2 * pixels.rows(), 3 * N);
    for (int const i : {0, 1, 2}) {
        A.middleCols(i * N, N) = InterleaveRowWise(points).rowwise().homogeneous();
    }

    for (Eigen::Index i{0}; i < pixels.rows(); ++i) {
        auto const pixel_i{pixels.row(i)};
        auto A_i{A.middleRows(i * 2, 2)};

        // Construct O^T, -w_i*X_i^T, y_i * X_i^T
        A_i.template block<1, N>(0, 0) = ArrayXd::Zero(N, 1).transpose();
        A_i.template block<1, N>(0, N) *= -1.0;
        A_i.template block<1, N>(0, 2 * N) *= pixel_i(1);

        // Construct w_i*X_i^T, O^T, -x_i * X_i^T
        A_i.template block<1, N>(1, 0) *= 1.0;
        A_i.template block<1, N>(1, N) = ArrayXd::Zero(N, 1).transpose();
        A_i.template block<1, N>(1, 2 * N) *= -pixel_i(0);
    }

    return A;
}  // LCOV_EXCL_LINE

// NOTE(Jack): I am not exactly sure the best notation here! I think this does a more generic operation than the name
// SolveForH suggests. It is used in Dlt22 to solve for the 3x3 H matrix and in Dlt23 to solve for the 3x4 P matrix. I
// think P is a homography therefore SolveForH is a name that covers both use cases.
template <int N>
std::optional<Eigen::Matrix<double, 3, N>> SolveForH(Eigen::Matrix<double, Eigen::Dynamic, 3 * N> const& A) {
    std::cout << 0 << std::endl;

    Eigen::JacobiSVD<MatrixXd> svd;
    svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    std::cout << "sizeof(Eigen::Index): " << sizeof(Eigen::Index) << '\n';
    std::cout << "sizeof(MatrixXd): " << sizeof(Eigen::MatrixXd) << '\n';
    std::cout << "sizeof(JacobiSVD): " << sizeof(Eigen::JacobiSVD<Eigen::MatrixXd>) << '\n';
    std::cout << "A rows/cols: " << A.rows() << ", " << A.cols() << '\n';
    std::cout << "V rows/cols: " << svd.matrixV().rows() << ", " << svd.matrixV().cols() << '\n';
    std::cout << "singular values size: " << svd.singularValues().size() << '\n';

    std::cout << 1 << std::endl;

    // WARN(Jack): This relative threshold multiplier was found heursitically by looking at one single dataset where I
    // was having trouble with a failed DLT solve causing a crazy frame pose and messing up the rest of the
    // optimization. If this value really applies everywhere I really do not know.
    // TODO(Jack): Reading about this threshold it seems like most places default to the value of doubel::epsilon, but
    // here we had to set the much large value of 5-5. Did we do something wrong? This fact really smells to me.
    double const relative_threshold{5e-5 * static_cast<double>(std::max(A.rows(), A.cols()))};
    std::cout << "relative_threshold: " << relative_threshold << std::endl;
    svd.setThreshold(relative_threshold);

    std::cout << 2 << std::endl;

    // TODO(Jack): Is less than the right condition to check here? What about enforcing actually equality?
    constexpr int expected_rank{3 * N - 1};
    std::cout << "expected_rank:" << expected_rank << std::endl;
    std::cout << "svd.rank():" << svd.rank() << std::endl;
    if (svd.rank() < expected_rank) {
        std::cout << 2.1 << std::endl;
        return std::nullopt;
    }

    std::cout << 3 << std::endl;

    // TODO (Jack): There has to be a more expressive way to pack .col(3*N -1) into P and select the column using 3*N -1
    Eigen::Matrix<double, 3, N> H;
    auto const last_col{svd.matrixV().col(3 * N - 1)};  // Has to be a better name than last col...
    H.row(0) = last_col.topRows<N>();
    H.row(1) = last_col.middleRows(N, N);
    H.row(2) = last_col.bottomRows<N>();

    std::cout << 4 << std::endl;

    return H;
}

}  // namespace reprojection::pnp