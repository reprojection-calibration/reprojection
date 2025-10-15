#include "camera_matrix_decomposition.hpp"

#include <gtest/gtest.h>

using namespace reprojection_calibration::pnp;

TEST(PnpCameraMatrixDecomposition, TestRqDecomposition) {
    Eigen::Matrix3d const M{{600, 0, 360}, {0, 600, 240}, {0, 0, 1}};

    auto const [R, Q]{RqDecomposition(M)};

    EXPECT_TRUE(R.isUpperTriangular());
    EXPECT_FLOAT_EQ((Q * Q.transpose()).diagonal().sum(), 3.0);  // Matrix is orthogonal - Q*Q^T returns identity
}

TEST(PnpCameraMatrixDecomposition, TestDecomposeMIntoRk) {
    Eigen::Matrix3d const M{{600, 0, 360}, {0, 600, 240}, {0, 0, 1}};

    auto const [K, R]{DecomposeMIntoKr(M)};

    EXPECT_TRUE(K.isUpperTriangular());
    EXPECT_EQ(K.diagonal().array().sign().sum(), 3);             // All diagonal entries of K must be positive
    EXPECT_FLOAT_EQ((R * R.transpose()).diagonal().sum(), 3.0);  // Orthogonal
    EXPECT_FLOAT_EQ(R.determinant(), 1.0);
}

TEST(PnpCameraMatrixDecomposition, TestCalculateCameraCenter) {
    Eigen::Matrix<double, 3, 4> P{{1, 0, 0, 100}, {0, 1, 0, 10}, {0, 0, 1, 1}};

    Eigen::Vector3d const camera_center{CalculateCameraCenter(P)};

    // "The camera center C is the point for which PC=0" - Quote from MVG section "6.2.4 Finding the camera center"
    EXPECT_TRUE((P * camera_center.homogeneous()).isApprox(Eigen::Vector3d::Zero()));
}