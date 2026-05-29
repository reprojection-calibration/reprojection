#include "spline/so3_spline.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "spline/constants.hpp"
#include "spline/spline_evaluation.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;
using namespace reprojection::spline;
using enum DerivativeOrder;

// Read the comments in TEST(Spline_r3Spline, TestEvaluateValidity) for more context.
TEST(SplineSo3Spline, TestEvaluateValidity) {
    MatrixNXd const empty{};
    CubicBSplineC3 const empty_spline{empty, TimeHandler{}};
    EXPECT_FALSE(EvaluateSpline<So3Spline>(empty_spline, 100, Null));

    CubicBSplineC3 const one_segment_spline{MatrixNKd::Zero(), TimeHandler{100, 5}};

    EXPECT_TRUE(EvaluateSpline<So3Spline>(one_segment_spline, 100, Null));
    EXPECT_FALSE(EvaluateSpline<So3Spline>(one_segment_spline, 105, Null));

    CubicBSplineC3 const two_segment_spline{Eigen::Matrix<double, 3, constants::order + 1>::Zero(),
                                            TimeHandler{100, 5}};
    EXPECT_TRUE(EvaluateSpline<So3Spline>(two_segment_spline, 105, Null));
}

CubicBSplineC3 BuildSo3TestSpline() {
    MatrixNKd so3_control_points;
    so3_control_points.col(0) = (Vector3d::Zero());
    for (int i{1}; i < constants::order; ++i) {
        // TODO(Jack): Evaluate if we can do any of the math directly in tangent space
        so3_control_points.col(i) =
            geometry::Log<double>(geometry::Exp<double>(so3_control_points.col(i - 1)) *
                                  geometry::Exp<double>(((static_cast<double>(i) / 10) * Vector3d::Ones()).eval()));
    }

    return CubicBSplineC3{so3_control_points, TimeHandler{0, 5'000'000}};
}

TEST(SplineSo3Spline, TestEvaluate) {
    CubicBSplineC3 const spline{BuildSo3TestSpline()};

    for (int i{0}; i < 5'000'000; i = i + 1'000'000) {
        auto const p_i{EvaluateSpline<So3Spline>(spline, i, Null)};
        ASSERT_TRUE(p_i.has_value());
    }

    Vector3d const p0{EvaluateSpline<So3Spline>(spline, 0, Null).value()};
    EXPECT_TRUE(p0.isApproxToConstant(
        0.11666666666666659));  // HEURISTIC! No theoretical testing strategy at this time - we have this here just so
                                // that we can detect changes to the implementation quickly (hopefully. )

    Vector3d const p4{EvaluateSpline<So3Spline>(spline, 4'000'000, Null).value()};
    EXPECT_TRUE(p4.isApproxToConstant(0.26866666666666655));
}

TEST(SplineSo3Spline, TestEvaluateVelocity) {
    CubicBSplineC3 const spline{BuildSo3TestSpline()};

    // RANDOM HEURISTIC TESTS!
    Vector3d const v0{EvaluateSpline<So3Spline>(spline, 0, First).value()};
    EXPECT_TRUE(v0.isApproxToConstant(30));

    Vector3d const v4{EvaluateSpline<So3Spline>(spline, 4'000'000, First).value()};
    EXPECT_TRUE(v4.isApproxToConstant(46));
}

TEST(SplineSo3Spline, TestEvaluateAcceleration) {
    CubicBSplineC3 const spline{BuildSo3TestSpline()};

    // RANDOM HEURISTIC TESTS! - but this does match exactly the change in velocity we see in the previous test :)
    Vector3d const a0{EvaluateSpline<So3Spline>(spline, 0, Second).value()};
    EXPECT_TRUE(a0.isApproxToConstant(4000));

    Vector3d const a4{EvaluateSpline<So3Spline>(spline, 4'000'000, Second).value()};
    EXPECT_TRUE(a4.isApproxToConstant(4000));
}

double Squared(double const x) { return x * x; }  // COPY PASTED

// NOTE(Jack): We use the same parabola data used for testing the r3 spline (see "Spline_r3Spline,
// TestTemplatedEvaluateOnParabola"). My hope was that it would be easier to interpret what is happening, but we are
// working with rotations here and I cannot see any obvious pattern, which does not totally surprise. That being said we
// some symmetry/alignment in the velocity and acceleration values. Maybe if we have time we can design a test that
// really is correct by inspection and not just heuristic.
TEST(SplineSo3Spline, TestTemplatedEvaluateOnParabola) {
    MatrixNKd const P1{{-1, -0.5, 0.5, 1},
                       {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)},
                       {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)}};
    double const u_middle{0.5};
    uint64_t const delta_t_ns{5'000'000};

    Vector3d const position{So3Spline::Evaluate<double, Null>(P1, u_middle, delta_t_ns)};
    EXPECT_TRUE(position.isApprox(Vector3d{0.1460482362445171, 0.3755842237411095, 0.39702710822143839}));

    Vector3d const velocity{So3Spline::Evaluate<double, First>(P1, u_middle, delta_t_ns)};
    EXPECT_TRUE(velocity.isApprox(Vector3d{171.27942373796074, -24.085617312239847, 25.44424511232922}));

    Vector3d const acceleration{So3Spline::Evaluate<double, Second>(P1, u_middle, delta_t_ns)};
    EXPECT_TRUE(acceleration.isApprox(Vector3d{279.89763763080555, 32038.115740062553, 28443.252525133488}));
}
