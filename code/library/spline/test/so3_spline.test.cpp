#include "spline/so3_spline.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "spline/constants.hpp"
#include "spline/spline_evaluation.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;
using namespace reprojection::spline;  // Use SOOO many times that we introduce this namespace

// Read the comments in TEST(Spline_r3Spline, TestEvaluateValidity) for more context.
TEST(SplineSo3Spline, TestEvaluateValidity) {
    CubicBSplineC3 const empty_spline{{100, 5}, {}};
    EXPECT_FALSE(EvaluateSpline<So3Spline>(empty_spline, 100));

    CubicBSplineC3 const one_segment_spline{{100, 5}, Matrix3Kd::Zero()};

    EXPECT_TRUE(EvaluateSpline<So3Spline>(one_segment_spline, 100));
    EXPECT_FALSE(EvaluateSpline<So3Spline>(one_segment_spline, 105));

    CubicBSplineC3 const two_segment_spline{{100, 5}, Eigen::Matrix<double, 3, constants::order + 1>::Zero()};
    EXPECT_TRUE(EvaluateSpline<So3Spline>(two_segment_spline, 105));
}

CubicBSplineC3 BuildSo3TestSpline() {
    std::vector<Vector3d> so3_control_points;
    so3_control_points.push_back(Vector3d::Zero());
    for (int i{1}; i < constants::order; ++i) {
        // TODO(Jack): Evaluate if we can do any of the math directly in tangent space
        so3_control_points.push_back(
            geometry::Log<double>(geometry::Exp<double>(so3_control_points.back()) *
                                  geometry::Exp<double>(((static_cast<double>(i) / 10) * Vector3d::Ones()).eval())));
    }

    return CubicBSplineC3{{100, 5}, so3_control_points};
}

TEST(SplineSo3Spline, TestEvaluate) {
    CubicBSplineC3 const spline{BuildSo3TestSpline()};

    // Check that all timestamps from 100 to 104 evaluate without error
    for (int i{0}; i < static_cast<int>(spline.time_handler_.delta_t_ns_); ++i) {
        auto const p_i{EvaluateSpline<So3Spline>(spline, 100 + i)};
        ASSERT_TRUE(p_i.has_value());
    }

    Vector3d const p0{EvaluateSpline<So3Spline>(spline, 100).value()};
    EXPECT_TRUE(p0.isApproxToConstant(
        0.11666666666666659));  // HEURISTIC! No theoretical testing strategy at this time - we have this here just so
                                // that we can detect changes to the implementation quickly (hopefully. )

    Vector3d const p4{EvaluateSpline<So3Spline>(spline, 104).value()};
    EXPECT_TRUE(p4.isApproxToConstant(0.26866666666666655));
}

TEST(SplineSo3Spline, TestEvaluateVelocity) {
    CubicBSplineC3 const spline{BuildSo3TestSpline()};

    // RANDOM HEURISTIC TESTS!
    Vector3d const v0{EvaluateSpline<So3Spline>(spline, 100, DerivativeOrder::First).value()};
    EXPECT_TRUE(v0.isApproxToConstant(0.03));

    Vector3d const v4{EvaluateSpline<So3Spline>(spline, 104, DerivativeOrder::First).value()};
    EXPECT_TRUE(v4.isApproxToConstant(0.046));
}

TEST(SplineSo3Spline, TestEvaluateAcceleration) {
    CubicBSplineC3 const spline{BuildSo3TestSpline()};

    // RANDOM HEURISTIC TESTS! - but this does match exactly the change in velocity we see in the previous test :)
    Vector3d const a0{EvaluateSpline<So3Spline>(spline, 100, DerivativeOrder::Second).value()};
    EXPECT_TRUE(a0.isApproxToConstant(0.004));

    Vector3d const a4{EvaluateSpline<So3Spline>(spline, 104, DerivativeOrder::Second).value()};
    EXPECT_TRUE(a4.isApproxToConstant(0.004));
}

double Squared(double const x) { return x * x; }  // COPY PASTED

// NOTE(Jack): We use the same parabola data used for testing the r3 spline (see "Spline_r3Spline,
// TestTemplatedEvaluateOnParabola"). My hope was that it would be easier to interpret what is happening, but we are
// working with rotations here and I cannot see any obvious pattern, which does not totally surprise. That being said we
// some symmetry/alignment in the velocity and acceleration values. Maybe if we have time we can design a test that
// really is correct by inspection and not just heuristic.
TEST(SplineSo3Spline, TestTemplatedEvaluateOnParabola) {
    Matrix3Kd const P1{{-1, -0.5, 0.5, 1},
                       {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)},
                       {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)}};
    double const u_middle{0.5};
    std::uint64_t const delta_t_ns{1};

    Vector3d const position{So3Spline::Evaluate<double, DerivativeOrder::Null>(P1, u_middle, delta_t_ns)};
    EXPECT_TRUE(position.isApprox(Vector3d{0.1460482362445171, 0.3755842237411095, 0.39702710822143839}));

    Vector3d const velocity{So3Spline::Evaluate<double, DerivativeOrder::First>(P1, u_middle, delta_t_ns)};
    EXPECT_TRUE(velocity.isApprox(Vector3d{0.8563971186898035, -0.1204280865611993, 0.12722122556164611}));

    Vector3d const acceleration{So3Spline::Evaluate<double, DerivativeOrder::Second>(P1, u_middle, delta_t_ns)};
    EXPECT_TRUE(acceleration.isApprox(Vector3d{0.0069974409407700944, 0.80095289350156396, 0.71108131312833733}));
}
