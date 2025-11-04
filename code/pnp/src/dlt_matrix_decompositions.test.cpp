#include "dlt_matrix_decompositions.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(PnpDltMatrixDecompositions, TestRqDecomposition) {
    Matrix3d const M{{600, 0, 360}, {0, 600, 240}, {0, 0, 1}};

    auto const [R, Q]{pnp::RqDecomposition(M)};

    EXPECT_TRUE(R.isUpperTriangular());
    EXPECT_FLOAT_EQ((Q * Q.transpose()).diagonal().sum(), 3.0);  // Matrix is orthogonal - Q*Q^T returns identity
}

TEST(PnpDltMatrixDecompositions, TestDecomposeMIntoRk) {
    Matrix3d const M{{600, 0, 360}, {0, 600, 240}, {0, 0, 1}};

    auto const [K, R]{pnp::DecomposeMIntoKr(M)};

    EXPECT_TRUE(K.isUpperTriangular());                          // Property of camera intrinsic matrix
    EXPECT_EQ(K.diagonal().array().sign().sum(), 3);             // All diagonal entries of K must be positive
    EXPECT_FLOAT_EQ((R * R.transpose()).diagonal().sum(), 3.0);  // Orthogonal
    EXPECT_FLOAT_EQ(R.determinant(), 1.0);
}

TEST(PnpDltMatrixDecompositions, TestCalculateCameraCenter) {
    Eigen::Matrix<double, 3, 4> P{{1, 0, 0, 100}, {0, 1, 0, 10}, {0, 0, 1, 1}};

    Vector3d const camera_center{pnp::CalculateCameraCenter(P)};

    // "The camera center C is the point for which PC=0" - Quote from MVG section "6.2.4 Finding the camera center"
    EXPECT_TRUE((P * camera_center.homogeneous()).isApprox(Vector3d::Zero()));
}

TEST(PnpDltMatrixDecompositions, TestDecomposeHIntoRt) {
    double const x{1.1};
    Matrix3d const H{{x, 0, 0.1}, {0, x, 0.2}, {0, 0, 0.3}};

    auto const [R, t]{pnp::DecomposeHIntoRt(H)};

    EXPECT_TRUE(R.isApprox(Matrix3d::Identity()));
    EXPECT_TRUE(t.isApprox(H.col(2) / x));
}