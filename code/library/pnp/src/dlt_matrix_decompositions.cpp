#include "dlt_matrix_decompositions.hpp"

#include "geometry/lie.hpp"

namespace reprojection::pnp {

// Inspired by https://www.physicsforums.com/threads/rq-decomposition-from-qr-decomposition.261739/
// We implement RQ decomposition in terms of Eigen's built in QR decomposition
std::tuple<Matrix3d, Matrix3d> RqDecomposition(Matrix3d const& matrix) {
    Matrix3d const reverse_rows{{0, 0, 1}, {0, 1, 0}, {1, 0, 0}};
    Matrix3d const reversed_matrix{reverse_rows * matrix};

    Eigen::HouseholderQR<Matrix3d> qr(reversed_matrix.transpose());
    Matrix3d const R{qr.matrixQR().triangularView<Eigen::Upper>()};
    Matrix3d const Q{qr.householderQ()};

    Matrix3d const R_star{reverse_rows * R.transpose() * reverse_rows};
    Matrix3d const Q_star{reverse_rows * Q.transpose()};

    return {R_star, Q_star};
}

// NOTE(Jack): MVG section "6.2.4 Decomposition of the camera matrix" refers to the first three columns of the camera
// matrix P as M.
// Adopted from https://ksimek.github.io/2012/08/14/decompose/
std::tuple<Matrix3d, Matrix3d> DecomposeMIntoKr(Matrix3d const& M) {
    const auto [K, R]{RqDecomposition(M)};

    // TODO(Jack): Fix hacky sign names here - what we are doing here is to make sure that the matrix K has all positive
    // diagonal values. See the referenced link (https://ksimek.github.io/2012/08/14/decompose/) for more details.
    Vector3d const sign{K.diagonal().array().sign()};
    Matrix3d const sign_mat{sign.asDiagonal()};

    Matrix3d const K_star{(K / std::abs(K(2, 2))) * sign_mat};
    Matrix3d R_star{sign_mat * R};
    if (R_star.determinant() < 0) {
        R_star *= -1;
    }

    return {K_star, R_star};
}

Vector3d CalculateCameraCenter(Matrix34d const& P) {
    double const x{P(Eigen::all, {1, 2, 3}).determinant()};
    double const y{-P(Eigen::all, {0, 2, 3}).determinant()};
    double const z{P(Eigen::all, {0, 1, 3}).determinant()};
    double const t{-P(Eigen::all, {0, 1, 2}).determinant()};

    return Vector3d{x, y, z} / t;
}

std::tuple<Matrix3d, Vector3d> DecomposeHIntoRt(Matrix3d const& H) {
    // The following discussion is based on the fact that all 3D points lie on one plane with Z=0. We start with H
    // which is the product of a scale lambda, K and [r1 r2 t]. But because we are working with ideal normalized image
    // coordinates (i.e. K = I) we can ignore K. Next, because r1 and r2 are columns or a rotation matrix we know their
    // norm must be one and therefore that fixes the scale lambda. Finally, we know that the columns of a rotation
    // matrix must be orthonormal, so we can calculate r3 as the cross product of the r1 and r2 vectors.
    //
    //      H = lambda * K * [r1 r2 t]
    //      H = lambda * [r1 r2 t]              <- K = I, it disappears
    //      H = [r1 r2 t]                       <- r1 and r2 are columns of a rotation matrix (l2 norm=1) - fixes lambda
    //      [R|t] = [r1 r2 (r1 x r2)|t]         <- rotation matrix columns are orthonormal - fixes r3

    // Use the average magnitude of both r1 and r2 to fix scale - more stable than using just r1 or r2 alone
    double const inv_lambda{(H.col(0).norm() + H.col(1).norm()) / 2};
    Matrix3d const H_star{H / inv_lambda};

    Matrix3d R;
    R.col(0) = H_star.col(0);
    R.col(1) = H_star.col(1);
    R.col(2) = H_star.col(0).cross(H_star.col(1));  // r3 is orthogonal to r1 and r2

    // WARN(Jack): This is a brute force method to get a "proper" rotation matrix - that being said it will probably
    // introduce an error. For a better solution we should "apply a polar decomposition, or orthogonalization of the
    // rotation matrix" for an optimal solution. https://www.continuummechanics.org/polardecomposition.html
    Matrix3d const R_star{reprojection::geometry::Exp((reprojection::geometry::Log(R)))};

    return {R_star, H_star.col(2)};
}

}  // namespace reprojection::pnp