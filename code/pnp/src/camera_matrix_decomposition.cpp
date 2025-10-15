#include "camera_matrix_decomposition.hpp"

namespace reprojection::pnp {

// Inspired by https://www.physicsforums.com/threads/rq-decomposition-from-qr-decomposition.261739/
// We implement RQ decomposition in terms of Eigen's built in QR decomposition
std::tuple<Eigen::Matrix3d, Eigen::Matrix3d> RqDecomposition(Eigen::Matrix3d const& matrix) {
    Eigen::Matrix3d const reverse_rows{{0, 0, 1}, {0, 1, 0}, {1, 0, 0}};
    Eigen::Matrix3d const reversed_matrix{reverse_rows * matrix};

    Eigen::HouseholderQR<Eigen::Matrix3d> qr(reversed_matrix.transpose());
    Eigen::Matrix3d const R{qr.matrixQR().triangularView<Eigen::Upper>()};
    Eigen::Matrix3d const Q{qr.householderQ()};

    Eigen::Matrix3d const R_star{reverse_rows * R.transpose() * reverse_rows};
    Eigen::Matrix3d const Q_star{reverse_rows * Q.transpose()};

    return {R_star, Q_star};
}

// NOTE(Jack): MVG section "6.2.4 Decomposition of the camera matrix" refers to the first three columns of the camera
// matrix P as M.
// Adopted from https://ksimek.github.io/2012/08/14/decompose/
std::tuple<Eigen::Matrix3d, Eigen::Matrix3d> DecomposeMIntoKr(Eigen::Matrix3d const& M) {
    const auto [K, R]{RqDecomposition(M)};

    // TODO(Jack): Fix hacky sign names here - what we are doing here is to make sure that the matrix K has all positive
    // diagonal values. See the referenced link (https://ksimek.github.io/2012/08/14/decompose/) for more details.
    Eigen::Vector3d const sign{K.diagonal().array().sign()};
    Eigen::Matrix3d const sign_mat{sign.asDiagonal()};

    Eigen::Matrix3d const K_star{(K / std::abs(K(2, 2))) * sign_mat};
    Eigen::Matrix3d R_star{sign_mat * R};
    if (R_star.determinant() < 0) {
        R_star *= -1;
    }

    return {K_star, R_star};
}

// NOTE(Jack):
Eigen::Vector3d CalculateCameraCenter(Eigen::Matrix<double, 3, 4> const& P) {
    double const x{P(Eigen::all, {1, 2, 3}).determinant()};
    double const y{-P(Eigen::all, {0, 2, 3}).determinant()};
    double const z{P(Eigen::all, {0, 1, 3}).determinant()};
    double const t{-P(Eigen::all, {0, 1, 2}).determinant()};

    return Eigen::Vector3d{x, y, z} / t;
}

}  // namespace reprojection::pnp