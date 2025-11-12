#include "spline/so3_spline.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "spline/constants.hpp"
#include "types/eigen_types.hpp"
#include "utilities_testing.hpp"

using namespace reprojection;

TEST(SplineSo3Spline, TestInvalidEvaluateConditions) {
    spline::SO3SplineState so3_spline{100, 5};
    EXPECT_EQ(spline::EvaluateSO3(115, so3_spline), std::nullopt);

    for (int i{0}; i < spline::constants::order; ++i) {
        so3_spline.control_points.push_back(Matrix3d::Identity());
    }

    EXPECT_NE(spline::EvaluateSO3(100, so3_spline), std::nullopt);
    EXPECT_EQ(spline::EvaluateSO3(105, so3_spline), std::nullopt);

    so3_spline.control_points.push_back(Matrix3d::Identity());
    EXPECT_NE(spline::EvaluateSO3(105, so3_spline), std::nullopt);
}

TEST(SplineSo3Spline, TestEvaluate) {
    uint64_t const delta_t_ns{5};
    spline::SO3SplineState so3_spline{100, delta_t_ns};

    so3_spline.control_points.push_back(geometry::Exp(Vector3d::Zero().eval()));
    for (int i{1}; i < spline::constants::order; ++i) {
        so3_spline.control_points.push_back(so3_spline.control_points.back() *
                                            geometry::Exp(((static_cast<double>(i) / 10) * Vector3d::Ones()).eval()));
    }

    // Heuristic test as we have no theoretical testing strategy at this time.
    for (int i{0}; i < static_cast<int>(delta_t_ns); ++i) {
        auto const p_i{spline::EvaluateSO3(100 + i, so3_spline)};
        ASSERT_TRUE(p_i.has_value());
        EXPECT_TRUE(spline::IsRotation(p_i.value()));
    }

    auto const p_0{spline::EvaluateSO3(100, so3_spline)};
    EXPECT_FLOAT_EQ(p_0.value().diagonal().sum(),
                    2.9593055);  // HEURISTIC! No theoretical testing strategy at this time - we have this here just so
                                 // that we can detect changes to the implementation quickly (hopefully. )
}

TEST(SplineSo3Spline, TestEvaluateVelocity) {
    uint64_t const delta_t_ns{5};
    spline::SO3SplineState so3_spline{100, delta_t_ns};
    so3_spline.control_points.push_back(geometry::Exp(Vector3d::Zero().eval()));

    EXPECT_EQ(spline::EvaluateSO3Velocity(100, so3_spline),
              std::nullopt);  // Not enough control_points yet to evaluate velocity

    for (int i{1}; i < spline::constants::order; ++i) {
        so3_spline.control_points.push_back(so3_spline.control_points.back() *
                                            geometry::Exp(((static_cast<double>(i) / 10) * Vector3d::Ones()).eval()));
    }

    // RANDOM HEURISTIC TESTS!
    Vector3d const v0{spline::EvaluateSO3Velocity(100, so3_spline).value()};
    EXPECT_TRUE(v0.isApproxToConstant(0.03));

    Vector3d const v4{spline::EvaluateSO3Velocity(104, so3_spline).value()};
    EXPECT_TRUE(v4.isApproxToConstant(0.046));
}

// TODO(Jack): We need a test fixture for the spline creation logic! It is copy and pasted many times.
TEST(SplineSo3Spline, TestEvaluateAcceleration) {
    uint64_t const delta_t_ns{5};
    spline::SO3SplineState so3_spline{100, delta_t_ns};
    so3_spline.control_points.push_back(geometry::Exp(Vector3d::Zero().eval()));

    EXPECT_EQ(spline::EvaluateSO3Acceleration(100, so3_spline),
              std::nullopt);  // Not enough control_points yet to evaluate acceleration

    for (int i{1}; i < spline::constants::order; ++i) {
        so3_spline.control_points.push_back(so3_spline.control_points.back() *
                                            geometry::Exp(((static_cast<double>(i) / 10) * Vector3d::Ones()).eval()));
    }

    // RANDOM HEURISTIC TESTS! - but this does match exactly the change in velocity we see in the previous test :)
    Vector3d const v0{spline::EvaluateSO3Acceleration(100, so3_spline).value()};
    EXPECT_TRUE(v0.isApproxToConstant(0.004));

    Vector3d const v4{spline::EvaluateSO3Acceleration(104, so3_spline).value()};
    EXPECT_TRUE(v4.isApproxToConstant(0.004));
}