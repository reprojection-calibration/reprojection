#include "ceres_kinematics.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(OptimizationCeresKinematics, TestTransformRigidBodyAccelerationIdentity) {
    // For an identity transform a_i will always equal a_j regardless of the rigid bodies motion
    Vector6d const tf_i_j{0, 0, 0, 0, 0, 0};

    for (int i{0}; i < 10; ++i) {
        Vector3d const omega_j{Vector3d::Random()};
        Vector3d const alpha_j{Vector3d::Random()};
        Vector3d const a_j{Vector3d::Random()};

        Vector3d const a_i{optimization::TransformRigidBodyAcceleration<double>(tf_i_j, omega_j, alpha_j, a_j)};

        EXPECT_TRUE(a_i.isApprox(a_j));
    }
}

TEST(OptimizationCeresKinematics, TestTransformRigidBodyAccelerationSameAxis) {
    // For two coordinate frames rotating along the same axis (x-axis here) the rotational acceleration will be the
    // same at both places.
    Vector6d const tf_i_j{0, 0, 0, 1, 0, 0};

    Vector3d const omega_j{1, 0, 0};
    Vector3d const alpha_j{1, 0, 0};

    for (int i{0}; i < 10; ++i) {
        Vector3d const a_j{Vector3d::Random()};

        Vector3d const a_i{optimization::TransformRigidBodyAcceleration<double>(tf_i_j, omega_j, alpha_j, a_j)};

        EXPECT_TRUE(a_i.isApprox(a_j));
    }
}

TEST(OptimizationCeresKinematics, TestTransformRigidBodyAccelerationHeuristic) {
    // Random data with a heuristic expected result so we can hopefully catch regressions.
    Vector6d const tf_i_j{0.1, 0.2, 0.3, 1, 2, 3};

    Vector3d const omega_j{1, 1, 1};
    Vector3d const alpha_j{1, 1, 1};
    Vector3d const a_j{1, 2, 3};

    Vector3d const a_i{optimization::TransformRigidBodyAcceleration<double>(tf_i_j, omega_j, alpha_j, a_j)};

    EXPECT_TRUE(a_i.isApprox(Vector3d{4.8889657223403367, 1.4466322506082459, 0.072589925481057083}));
}
