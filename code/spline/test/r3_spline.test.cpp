#include "spline/r3_spline.hpp"

#include <gtest/gtest.h>

#include "spline/constants.hpp"
#include "spline/spline_evaluation.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

// TODO(Jack): Are our units of time right? Or are we accidentally calculating everything in m/ns?

TEST(Spline_r3Spline, TestInvalidEvaluateConditions) {
    // Completely empty spline
    spline::CubicBSplineC3 r3_spline{100, 5};
    EXPECT_EQ(spline::EvaluateSpline<spline::R3SplineEvaluation>(115, r3_spline), std::nullopt);

    // Add four control_points which means we can ask for evaluations within the one valid time segment.
    for (int i{0}; i < spline::constants::order; ++i) {
        r3_spline.control_points.push_back(Eigen::Vector3d::Zero());
    }

    EXPECT_NE(spline::EvaluateSpline<spline::R3SplineEvaluation>(100, r3_spline),
              std::nullopt);  // Inside first time segment - valid
    EXPECT_EQ(spline::EvaluateSpline<spline::R3SplineEvaluation>(105, r3_spline),
              std::nullopt);  // Outside first time segment - invalid

    // Add one more control_point to see that we can now do a valid evaluation in a second time segment
    r3_spline.control_points.push_back(Eigen::Vector3d::Zero());
    EXPECT_NE(spline::EvaluateSpline<spline::R3SplineEvaluation>(105, r3_spline), std::nullopt);
}

TEST(Spline_r3Spline, TestEvaluate) {
    // Fill spline with four control points which for a cubic b-spline gives us one valid time segment.
    spline::CubicBSplineC3 r3_spline{100, 5};
    for (int i{0}; i < spline::constants::order; ++i) {
        r3_spline.control_points.push_back(i * Eigen::Vector3d::Ones());
    }

    // Position
    auto const p_0{spline::EvaluateSpline<spline::R3SplineEvaluation>(100, r3_spline)};
    ASSERT_TRUE(p_0.has_value());
    EXPECT_TRUE(p_0.value().isApproxToConstant(1));

    // Velocity
    auto const v_0{spline::EvaluateSpline<spline::R3SplineEvaluation>(100, r3_spline, spline::DerivativeOrder::First)};
    ASSERT_TRUE(v_0.has_value());
    EXPECT_TRUE(v_0.value().isApproxToConstant(0.2));  // 1m/5ns

    // Acceleration
    auto const a_0{spline::EvaluateSpline<spline::R3SplineEvaluation>(100, r3_spline, spline::DerivativeOrder::Second)};
    ASSERT_TRUE(a_0.has_value());
    EXPECT_TRUE(a_0.value().isApproxToConstant(0));  // Straight line has no acceleration

    // Add one more element and test the first position in that second time segment
    r3_spline.control_points.push_back(4 * Vector3d::Ones());
    auto const p_1{spline::EvaluateSpline<spline::R3SplineEvaluation>(105, r3_spline)};
    ASSERT_TRUE(p_1.has_value());
    EXPECT_TRUE(p_1.value().isApproxToConstant(2));
}

// This test uses a linear set of four control points going from 0 to 3 to show the basic ranges of how a cubic b-spline
// evaluates. The main takeaway is that the output values will be found between the second and third control point. The
// first and fourth control point influence the value, but are outside the range of the generated values. In the test
// below we show this by evaluating the spline with the minimum set of control points (4) and see that when u=0 we get
// back the point 2, and when u=0.99999 we get back the point 3. Values in between 0 and 1 and 3 and 4 cannot be
// estimated.
TEST(Spline_r3Spline, TestTemplatedEvaluateOnLine) {
    spline::Matrix3Kd const P1{{0, 1, 2, 3}, {0, 1, 2, 3}, {0, 1, 2, 3}};
    std::uint64_t const delta_t_ns{5};  // No effect when spline::DerivativeOrder::Null (5^0 = 1)

    Vector3d const position_1{
        spline::R3SplineEvaluation::Evaluate<double, spline::DerivativeOrder::Null>(P1, 0, delta_t_ns)};
    EXPECT_TRUE(position_1.isApproxToConstant(1));

    // NOTE(Jack): The spline time u include [0,1), therefore we have to set u_i=0.999999 something, instead of just 1.
    Vector3d const position_2{
        spline::R3SplineEvaluation::Evaluate<double, spline::DerivativeOrder::Null>(P1, 0.999999, delta_t_ns)};
    EXPECT_TRUE(position_2.isApproxToConstant(2, 1e-6));

    // Shift the control points now to start at 1 and end at 4 manually by creating a new P, and see that the spline now
    // starts at 2 instead of 1.
    spline::Matrix3Kd const P2{{1, 2, 3, 4}, {1, 2, 3, 4}, {1, 2, 3, 4}};
    Vector3d const position_3{
        spline::R3SplineEvaluation::Evaluate<double, spline::DerivativeOrder::Null>(P2, 0, delta_t_ns)};
    EXPECT_TRUE(position_3.isApproxToConstant(2));
}

double Squared(double const x) { return x * x; }  // COPY PASTED

// If you really want to understand this test you need to print out the P matrix below and go to
// https://nurbscalculator.in/ to plot it (don't forget to set it to a degree 3 B-Spline!). There you will see that it
// kind of looks like a parabola aligned along the x-axis. You can set the value of u to 0.5 to get the expected
// position you see below, and then can manually inspect the graph to convince yourself that the first and second
// derivatives (i.e. velocity and acceleration) make sense :)
//
// We added this test because we wanted a test where the derivatives could somehow be roughly inferred and understood,
// and where the second derivative was not just zero like it is for the linear control point case. Also note that the
// control points are generated like a parabola, but that does not necessarily mean the spline is a parabola itself!
// Look at the nurbs calculator viewer to try to understand what is going on here!
TEST(Spline_r3Spline, TestTemplatedEvaluateOnParabola) {
    spline::Matrix3Kd const P1{{-1, -0.5, 0.5, 1},
                               {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)},
                               {Squared(-1), Squared(-0.5), Squared(0.5), Squared(1)}};
    double const u_middle{0.5};  // Middle/center and therefore "bottom" of the "parabola" specified by control points P
    std::uint64_t const delta_t_ns{1};  // Setting to 1 avoids time dependent distortion to the velocity/acceleration

    Vector3d const position{
        spline::R3SplineEvaluation::Evaluate<double, spline::DerivativeOrder::Null>(P1, u_middle, delta_t_ns)};
    EXPECT_TRUE(position.isApprox(Vector3d{
        0, 0.28125, 0.28125}));  // Aligned and centered on the x-axis, value taken from nurbs calculator linked above.

    Vector3d const velocity{
        spline::R3SplineEvaluation::Evaluate<double, spline::DerivativeOrder::First>(P1, u_middle, delta_t_ns)};
    EXPECT_TRUE(velocity.isApprox(Vector3d{0.875, 0, 0}));  // Only x-axis motion at base of aligned parabola

    Vector3d const acceleration{
        spline::R3SplineEvaluation::Evaluate<double, spline::DerivativeOrder::Second>(P1, u_middle, delta_t_ns)};
    EXPECT_TRUE(acceleration.isApprox(Vector3d{0, 0.75, 0.75}));  // TODO(Jack): Explain why this makes sense!
}
