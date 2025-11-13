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
            geometry::Log<double>(geometry::Exp<double>(so3_spline.control_points.back()) *
                                  geometry::Exp<double>(((static_cast<double>(i) / 10) * Vector3d::Ones()).eval())));
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

TEST(SplineSo3Spline, TestEvaluateAcceleration) {
    spline::So3SplineState const spline{BuildTestSpline()};

    // RANDOM HEURISTIC TESTS! - but this does match exactly the change in velocity we see in the previous test :)
    Vector3d const a0{spline::EvaluateSo3(100, spline, spline::DerivativeOrder::Second).value()};
    EXPECT_TRUE(a0.isApproxToConstant(0.004));

    Vector3d const a4{spline::EvaluateSo3(104, spline, spline::DerivativeOrder::Second).value()};
    EXPECT_TRUE(a4.isApproxToConstant(0.004));
}

double Squared(double const x) { return x * x; }  // COPY PASTED

// NOTE(Jack): We use the same parabola data used for testing the r3 spline (see "Spline_r3Spline,
// TestTemplatedEvaluateOnParabola"). My hope was that it would be easier to interpret what is happening, but we are
// working with rotations here and I cannot see any obvious pattern, which does not totally surprise. That being said we
// some symmetry/alignment in the velocity and acceleration values. Maybe if we have time we can design a test that
// really is correct by inspection and not just heuristic.
TEST(SplineSo3Spline, TestTemplatedEvaluateOnParabola) {
    spline::Matrix3Kd const P1{{-1, -0.5, 0.5, 1},
                               {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)},
                               {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)}};
    double const u_middle{0.5};
    std::uint64_t const delta_t_ns{1};

    Vector3d const position{
        spline::So3SplineEvaluation::Evaluate<double, spline::DerivativeOrder::Null>(P1, u_middle, delta_t_ns)};
    EXPECT_TRUE(position.isApprox(Vector3d{0.1460482362445171, 0.3755842237411095, 0.39702710822143839}));

    Vector3d const velocity{
        spline::So3SplineEvaluation::Evaluate<double, spline::DerivativeOrder::First>(P1, u_middle, delta_t_ns)};
    EXPECT_TRUE(velocity.isApprox(Vector3d{0.8563971186898035, -0.1204280865611993, 0.12722122556164611}));

    Vector3d const acceleration{
        spline::So3SplineEvaluation::Evaluate<double, spline::DerivativeOrder::Second>(P1, u_middle, delta_t_ns)};
    EXPECT_TRUE(acceleration.isApprox(Vector3d{0.0069974409407700944, 0.80095289350156396, 0.71108131312833733}));
}
