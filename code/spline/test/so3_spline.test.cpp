#include "spline/so3_spline.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "spline/constants.hpp"
#include "types/eigen_types.hpp"
#include "utilities_testing.hpp"

using namespace reprojection;

TEST(SplineSo3Spline, TestInvalidEvaluateConditions) {
    spline::So3SplineState so3_spline{100, 5};
    EXPECT_EQ(spline::EvaluateSo3(115, so3_spline), std::nullopt);

    for (int i{0}; i < spline::constants::order; ++i) {
        so3_spline.control_points.push_back(Vector3d::Zero());
    }

    EXPECT_NE(spline::EvaluateSo3(100, so3_spline), std::nullopt);
    EXPECT_EQ(spline::EvaluateSo3(105, so3_spline), std::nullopt);

    so3_spline.control_points.push_back(Vector3d::Zero());
    EXPECT_NE(spline::EvaluateSo3(105, so3_spline), std::nullopt);
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

    // Check that all timestamps from 100 to 104 evaluate without error
    for (int i{0}; i < static_cast<int>(spline.time_handler.delta_t_ns_); ++i) {
        auto const p_i{spline::EvaluateSo3(100 + i, spline)};
        ASSERT_TRUE(p_i.has_value());
    }

    Vector3d const p0{spline::EvaluateSo3(100, spline).value()};
    EXPECT_TRUE(p0.isApproxToConstant(
        0.11666666666666659));  // HEURISTIC! No theoretical testing strategy at this time - we have this here just so
                                // that we can detect changes to the implementation quickly (hopefully. )

    Vector3d const p4{spline::EvaluateSo3(104, spline).value()};
    EXPECT_TRUE(p4.isApproxToConstant(0.26866666666666655));
}

TEST(SplineSo3Spline, TestEvaluateVelocity) {
    spline::So3SplineState const spline{BuildTestSpline()};

    // RANDOM HEURISTIC TESTS!
    Vector3d const v0{spline::EvaluateSo3(100, spline, spline::DerivativeOrder::First).value()};
    EXPECT_TRUE(v0.isApproxToConstant(0.03));

    Vector3d const v4{spline::EvaluateSo3(104, spline, spline::DerivativeOrder::First).value()};
    EXPECT_TRUE(v4.isApproxToConstant(0.046));
}

// TODO(Jack): We need a test fixture for the spline creation logic! It is copy and pasted many times.
TEST(SplineSo3Spline, TestEvaluateAcceleration) {
    spline::So3SplineState const spline{BuildTestSpline()};

    // RANDOM HEURISTIC TESTS! - but this does match exactly the change in velocity we see in the previous test :)
    Vector3d const a0{spline::EvaluateSo3(100, spline, spline::DerivativeOrder::Second).value()};
    EXPECT_TRUE(a0.isApproxToConstant(0.004));

    Vector3d const a4{spline::EvaluateSo3(104, spline, spline::DerivativeOrder::Second).value()};
    EXPECT_TRUE(a4.isApproxToConstant(0.004));
}