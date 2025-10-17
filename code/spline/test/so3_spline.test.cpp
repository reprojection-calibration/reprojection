#include "spline/so3_spline.hpp"

#include <gtest/gtest.h>

#include "constants.hpp"
#include "utilities_testing.hpp"
#include "geometry/lie.hpp"

using namespace reprojection;
using namespace reprojection::spline;

TEST(SplineSo3Spline, TestInvalidEvaluateConditions) {
    // Completely empty spline
    So3Spline so3_spline{100, 5};
    EXPECT_EQ(so3_spline.Evaluate(115), std::nullopt);

    // Add four knots which means we can ask for evaluations within the one time segment at the very start of the spline
    for (int i{0}; i < constants::k; ++i) {
        so3_spline.knots_.push_back(Eigen::Matrix3d::Identity());
    }

    EXPECT_NE(so3_spline.Evaluate(100), std::nullopt);  // Inside first time segment - valid
    EXPECT_EQ(so3_spline.Evaluate(105), std::nullopt);  // Outside first time segment - invalid

    // Add one more knot to see that we can now do a valid evaluation in the second time segment
    so3_spline.knots_.push_back(Eigen::Matrix3d::Identity());
    EXPECT_NE(so3_spline.Evaluate(105), std::nullopt);
}

TEST(SplineSo3Spline, TestEvaluate) {
    uint64_t const delta_t_ns{5};
    So3Spline so3_spline{100, delta_t_ns};
    so3_spline.knots_.push_back(geometry::Exp(Eigen::Vector3d::Zero().eval()));

    for (int i{1}; i < constants::k; ++i) {
        so3_spline.knots_.push_back(so3_spline.knots_.back() *
                                    geometry::Exp(((static_cast<double>(i) / 10) * Eigen::Vector3d::Ones()).eval()));
    }

    // Heuristic test as we have no theoretical testing strategy at this time.
    for (int i{0}; i < static_cast<int>(delta_t_ns); ++i) {
        auto const p_i{so3_spline.Evaluate(100 + i)};
        ASSERT_TRUE(p_i.has_value());
        EXPECT_TRUE(IsRotation(p_i.value()));
    }

    auto const p_0{so3_spline.Evaluate(100)};
    EXPECT_FLOAT_EQ(p_0.value().diagonal().sum(),
                    2.9593055);  // HEURISTIC! No theoretical testing strategy at this time - we have this here just so
                                 // that we can detect changes to the implementation quickly (hopefully. )
}

TEST(SplineSo3Spline, TestEvaluateVelocity) {
    uint64_t const delta_t_ns{5};
    So3Spline so3_spline{100, delta_t_ns};
    so3_spline.knots_.push_back(geometry::Exp(Eigen::Vector3d::Zero().eval()));

    EXPECT_EQ(so3_spline.EvaluateVelocity(100), std::nullopt);  // Not enough knots yet to evaluate velocity

    for (int i{1}; i < constants::k; ++i) {
        so3_spline.knots_.push_back(so3_spline.knots_.back() *
                                    geometry::Exp(((static_cast<double>(i) / 10) * Eigen::Vector3d::Ones()).eval()));
    }

    // RANDOM HEURISTIC TESTS!
    Eigen::Vector3d const v0{so3_spline.EvaluateVelocity(100).value()};
    EXPECT_TRUE(v0.isApproxToConstant(0.03));

    Eigen::Vector3d const v4{so3_spline.EvaluateVelocity(104).value()};
    EXPECT_TRUE(v4.isApproxToConstant(0.046));
}

// TODO(Jack): We need a test fixture for the spline creation logic! It is copy and pasted many times.
TEST(SplineSo3Spline, TestEvaluateAcceleration) {
    uint64_t const delta_t_ns{5};
    So3Spline so3_spline{100, delta_t_ns};
    so3_spline.knots_.push_back(geometry::Exp(Eigen::Vector3d::Zero().eval()));

    EXPECT_EQ(so3_spline.EvaluateAcceleration(100), std::nullopt);  // Not enough knots yet to evaluate acceleration

    for (int i{1}; i < constants::k; ++i) {
        so3_spline.knots_.push_back(so3_spline.knots_.back() *
                                    geometry::Exp(((static_cast<double>(i) / 10) * Eigen::Vector3d::Ones()).eval()));
    }

    // RANDOM HEURISTIC TESTS! - but this does match exactly the change in velocity we see in the previous test :)
    Eigen::Vector3d const v0{so3_spline.EvaluateAcceleration(100).value()};
    EXPECT_TRUE(v0.isApproxToConstant(0.004));

    Eigen::Vector3d const v4{so3_spline.EvaluateAcceleration(104).value()};
    EXPECT_TRUE(v4.isApproxToConstant(0.004));
}