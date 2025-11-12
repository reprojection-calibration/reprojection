#include "spline/so3_spline.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "spline/constants.hpp"
#include "types/eigen_types.hpp"
#include "utilities_testing.hpp"

using namespace reprojection;

TEST(SplineSo3Spline, TestInvalidEvaluateConditions) {
    spline::So3SplineState so3_spline{100, 5};
    EXPECT_EQ(spline::So3SplineEvaluation::Evaluate(115, so3_spline), std::nullopt);

    for (int i{0}; i < spline::constants::order; ++i) {
        so3_spline.control_points.push_back(Vector3d::Zero());
    }

    EXPECT_NE(spline::So3SplineEvaluation::Evaluate(100, so3_spline), std::nullopt);
    EXPECT_EQ(spline::So3SplineEvaluation::Evaluate(105, so3_spline), std::nullopt);

    so3_spline.control_points.push_back(Vector3d::Zero());
    EXPECT_NE(spline::So3SplineEvaluation::Evaluate(105, so3_spline), std::nullopt);
}

spline::So3SplineState BuildTestSpline() {
    uint64_t const delta_t_ns{5};
    spline::So3SplineState so3_spline{100, delta_t_ns};

    so3_spline.control_points.push_back(Vector3d::Zero());
    for (int i{1}; i < spline::constants::order; ++i) {
        // TODO(Jack): Evaluate if we can do any of the math directly in tangent space
        so3_spline.control_points.push_back(
            geometry::Log(geometry::Exp(so3_spline.control_points.back()) *
                          geometry::Exp(((static_cast<double>(i) / 10) * Vector3d::Ones()).eval())));
    }

    return so3_spline;
}

TEST(SplineSo3Spline, TestEvaluate) {
    spline::So3SplineState const spline{BuildTestSpline()};

    // Heuristic test as we have no theoretical testing strategy at this time.
    for (int i{0}; i < static_cast<int>(spline.time_handler.delta_t_ns_); ++i) {
        auto const p_i{spline::So3SplineEvaluation::Evaluate(100 + i, spline)};
        ASSERT_TRUE(p_i.has_value());
    }

    auto const p_0{spline::So3SplineEvaluation::Evaluate(100, spline)};
    // TODO(Jack): Update to just check the vector. Current value comes from when the evaluate method returned a
    // rotation matrix.
    EXPECT_FLOAT_EQ(geometry::Exp(p_0.value()).diagonal().sum(),
                    2.9593055);  // HEURISTIC! No theoretical testing strategy at this time - we have this here just so
                                 // that we can detect changes to the implementation quickly (hopefully. )
}

TEST(SplineSo3Spline, TestEvaluateVelocity) {
    spline::So3SplineState const spline{BuildTestSpline()};

    // RANDOM HEURISTIC TESTS!
    Vector3d const v0{spline::So3SplineEvaluation::EvaluateVelocity(100, spline).value()};
    EXPECT_TRUE(v0.isApproxToConstant(0.03));

    Vector3d const v4{spline::So3SplineEvaluation::EvaluateVelocity(104, spline).value()};
    EXPECT_TRUE(v4.isApproxToConstant(0.046));
}

// TODO(Jack): We need a test fixture for the spline creation logic! It is copy and pasted many times.
TEST(SplineSo3Spline, TestEvaluateAcceleration) {
    spline::So3SplineState const spline{BuildTestSpline()};

    // RANDOM HEURISTIC TESTS! - but this does match exactly the change in velocity we see in the previous test :)
    Vector3d const v0{spline::So3SplineEvaluation::EvaluateAcceleration(100, spline).value()};
    EXPECT_TRUE(v0.isApproxToConstant(0.004));

    Vector3d const v4{spline::So3SplineEvaluation::EvaluateAcceleration(104, spline).value()};
    EXPECT_TRUE(v4.isApproxToConstant(0.004));
}