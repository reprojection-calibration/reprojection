#include "ceres_kinematics.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(OptimizationCeresKinematics, TestTransformRigidBodyAcceleration) {
    // For an identity transfrom a_i will always equal a_j regardless of the rigid bodies motion
    Vector6d const tf_i_j{0, 0, 0, 0, 0, 0};

    for (int i{0}; i < 10; ++i) {
        Vector3d const omega_j{Vector3d::Random()};
        Vector3d const alpha_j{Vector3d::Random()};
        Vector3d const a_j{Vector3d::Random()};

        Vector3d const a_i{optimization::TransformRigidBodyAcceleration<double>(tf_i_j, omega_j, alpha_j, a_j)};
        EXPECT_TRUE(a_i.isApprox(a_j));
    }
}
