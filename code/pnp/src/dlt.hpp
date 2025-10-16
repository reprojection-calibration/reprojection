#pragma once

#include <Eigen/Dense>

#include "matrix_utilities.hpp"

namespace reprojection::pnp {

// NOTE(Jack): We know the K matrix ahead of time, at least that is then assumption the opencv pnp implementation
// makes, therefore the question is; Can we somehow use that knowledge to make our DLT better, and eliminate that we
// solve for K here? Or should we just follow the law of useful return and return T and K? For now we will do the
// latter but let's keep our eyes peeled for possible simplifications in the future.
std::tuple<Eigen::Isometry3d, Eigen::Matrix3d> Dlt(Eigen::MatrixX2d const& pixels, Eigen::MatrixX3d const& points);

// TODO(Jack): Fix comments to reflect new templated nature
// The 2n x 12 matrix assembled by stacking up the constraints from (MVG Eq. 7.2)
// NOTE(Jack): I am not gonna test this because I hope this function changes soon, see the note in the function.
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

// TODO(Jack): This solves for P when N=4 but solves for H when N=3 !!! Consider name change
template <int N>
Eigen::Matrix<double, 3, N> SolveForP(Eigen::Matrix<double, Eigen::Dynamic, 3 * N> const& A) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // TODO (Jack): There has to be a more expressive way to pack .col(3*N -1) into P and select the column using 3*N -1
    Eigen::Matrix<double, 3, N> P;
    auto const last_col{svd.matrixV().col(3 * N - 1)};  // Has to be a better name than last col...
    P.row(0) = last_col.topRows(N);
    P.row(1) = last_col.middleRows(N, N);
    P.row(2) = last_col.bottomRows(N);

    return P;
}

}  // namespace reprojection::pnp