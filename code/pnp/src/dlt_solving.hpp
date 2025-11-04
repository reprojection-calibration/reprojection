#pragma once

#include <Eigen/Core>

#include "matrix_utilities.hpp"

namespace reprojection::pnp {

// NOTE(Jack): Constructs the constraint matrix for the dlt problem given two sets of points, it is templated to handle
// both the plane to plane (Dlt22) and plane to points (Dlt23) case. In the Dlt22 case N=3 and A is 2nx9. In the Dlt23
// case N=4 and A is 2nx12.
// The math behind it for the Dlt23 case can be found in "Multiple View Geometry in computer vision", particularly be
// noting that this function stacks Eq. 7.2 to form the A matrix.
template <int N>
Eigen::Matrix<double, Eigen::Dynamic, 3 * N> ConstructA(Eigen::MatrixX2d const& pixels,
                                                        Eigen::Matrix<double, Eigen::Dynamic, N - 1> const& points) {
    Eigen::Matrix<double, Eigen::Dynamic, 3 * N> A(2 * pixels.rows(), 3 * N);
    for (int const i : {0, 1, 2}) {
        A.middleCols(i * N, N) = InterleaveRowWise(points).rowwise().homogeneous();
    }

    for (Eigen::Index i{0}; i < pixels.rows(); ++i) {
        auto const pixel_i{pixels.row(i)};
        auto A_i{A.middleRows(i * 2, 2)};

        // Construct O^T, -w_i*X_i^T, y_i * X_i^T
        A_i.template block<1, N>(0, 0) = Eigen::ArrayXXd::Zero(1, N);
        A_i.template block<1, N>(0, N) *= -1.0;
        A_i.template block<1, N>(0, 2 * N) *= pixel_i(1);

        // Construct w_i*X_i^T, O^T, -x_i * X_i^T
        A_i.template block<1, N>(1, 0) *= 1.0;
        A_i.template block<1, N>(1, N) = Eigen::ArrayXXd::Zero(1, N);
        A_i.template block<1, N>(1, 2 * N) *= -pixel_i(0);
    }

    return A;
}  // LCOV_EXCL_LINE

// NOTE(Jack): I am not exactly sure the best notation here! I think this does a more generic operation than the name
// SolveForH suggests. It is used in Dlt22 to solve for the 3x3 H matrix and in Dlt23 to solve for the 3x4 P matrix. I
// think P is a homography therefore SolveForH is a name that covers both use cases.
template <int N>
Eigen::Matrix<double, 3, N> SolveForH(Eigen::Matrix<double, Eigen::Dynamic, 3 * N> const& A) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // TODO (Jack): There has to be a more expressive way to pack .col(3*N -1) into P and select the column using 3*N -1
    Eigen::Matrix<double, 3, N> H;
    auto const last_col{svd.matrixV().col(3 * N - 1)};  // Has to be a better name than last col...
    H.row(0) = last_col.topRows(N);
    H.row(1) = last_col.middleRows(N, N);
    H.row(2) = last_col.bottomRows(N);

    return H;
}

}  // namespace reprojection::pnp