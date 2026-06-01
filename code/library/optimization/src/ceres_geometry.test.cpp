#include "ceres_geometry.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(OptimizationCeresGeometry, TestTransformPointsTranslation) {
    Vector6d const tf{0, 0, 0, 1, 2, 3};  // Translation only
    Vector3d const point{5, 10, 15};

    Vector3d const transformed_point{optimization::TransformPoint<double>(tf, point)};

    EXPECT_FLOAT_EQ(transformed_point[0], 6.0);
    EXPECT_FLOAT_EQ(transformed_point[1], 12.0);
    EXPECT_FLOAT_EQ(transformed_point[2], 18.0);
}

TEST(OptimizationCeresGeometry, TestTransformPointsRotation) {
    Vector6d const tf{0, 0, M_PI_2, 0, 0, 0};  // Rotation only
    Vector3d const point{5, 10, 15};

    Vector3d const transformed_point{optimization::TransformPoint<double>(tf, point)};

    EXPECT_FLOAT_EQ(transformed_point[0], -10.0);
    EXPECT_FLOAT_EQ(transformed_point[1], 5.0);
    EXPECT_FLOAT_EQ(transformed_point[2], 15.0);
}

TEST(OptimizationCeresGeometry, TestTransformRigidBodyAcceleration) {
    struct TestCase {
        Vector6d tf_i_j;
        Vector3d omega_j;
        Vector3d alpha_j;
        Vector3d acc_j;
        Vector3d acc_i;
    };

    std::vector<TestCase> const test_cases{
        // The null case
        {Vector6d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero()},
        // The coincident case
        {Vector6d::Zero(), Vector3d::Random(), Vector3d::Random(), Vector3d{1, 2, 3}, Vector3d{1, 2, 3}},
        // Rotation velocity/acceleration aligned with the axis of displacement does not contribute to acceleration.
        {Vector6d{0, 0, 0, 1, 0, 0}, Vector3d{1, 0, 0}, Vector3d{1, 0, 0}, Vector3d{1, 2, 3}, Vector3d{1, 2, 3}},
        // Centripetal acceleration only (i.e. the ω2r term)
        {Vector6d{0, 0, 0, 1, 0, 0}, Vector3d{0, 0, 2}, Vector3d::Zero(), Vector3d::Zero(), Vector3d{-4, 0, 0}},
        // Tangential acceleration only
        {Vector6d{0, 0, 0, 1, 0, 0}, Vector3d::Zero(), Vector3d{0, 0, 1}, Vector3d::Zero(), Vector3d{0, 1, 0}},
        // Extrinsic rotation only
        {Vector6d{0, 0, M_PI / 2, 0, 0, 0}, Vector3d::Zero(), Vector3d::Zero(), Vector3d{1, 2, 3}, Vector3d{-2, 1, 3}},
        // Random all axis heuristic because I got too lazy to test all the rotation and rotation+translation cases :(
        {Vector6d{M_PI / 2, M_PI / 2, M_PI / 2, 1, 1, 1}, Vector3d{1, 2, 3}, Vector3d{1, 2, 3}, Vector3d::Zero(),
         Vector3d{5.0967441810920171, -6.6561475737114524, -4.44059660738057}},
    };

    for (auto const& case_i : test_cases) {
        auto const acc_i{optimization::TransformRigidBodyAcceleration<double>(case_i.tf_i_j, case_i.omega_j,
                                                                              case_i.alpha_j, case_i.acc_j)};
        EXPECT_TRUE(acc_i.isApprox(case_i.acc_i))
            << "Result: " << acc_i.transpose() << " expected: " << case_i.acc_i.transpose() << std::endl;
    }
}