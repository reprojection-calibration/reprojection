#include "spline/r3_spline.hpp"

#include <gtest/gtest.h>

#include "spline/constants.hpp"
#include "spline/types.hpp"
#include "spline/utilities.hpp"

using namespace reprojection;

TEST(Spline_r3Spline, TestInvalidEvaluateConditions) {
    // Completely empty spline
    spline::r3Spline r3_spline{100, 5};
    EXPECT_EQ(r3_spline.Evaluate(115), std::nullopt);

    // Add four control_points which means we can ask for evaluations within the one time segment at the very start of
    // the spline
    for (int i{0}; i < spline::constants::order; ++i) {
        r3_spline.control_points_.push_back(Eigen::Vector3d::Zero());
    }

    EXPECT_NE(r3_spline.Evaluate(100), std::nullopt);  // Inside first time segment - valid
    EXPECT_EQ(r3_spline.Evaluate(105), std::nullopt);  // Outside first time segment - invalid

    // Add one more control_point to see that we can now do a valid evaluation in the second time segment
    r3_spline.control_points_.push_back(Eigen::Vector3d::Zero());
    EXPECT_NE(r3_spline.Evaluate(105), std::nullopt);
}

TEST(Spline_r3Spline, TestEvaluate) {
    spline::r3Spline r3_spline{100, 5};
    for (int i{0}; i < spline::constants::order; ++i) {
        r3_spline.control_points_.push_back(i * Eigen::Vector3d::Ones());
    }

    // Test three elements in the first and only valid time segment
    auto const p_0{r3_spline.Evaluate(100)};
    ASSERT_TRUE(p_0.has_value());
    EXPECT_TRUE(p_0.value().isApproxToConstant(1));

    auto const p_1{r3_spline.Evaluate(101)};
    ASSERT_TRUE(p_1.has_value());
    EXPECT_TRUE(p_1.value().isApproxToConstant(1.2));

    auto const p_2{r3_spline.Evaluate(102)};
    ASSERT_TRUE(p_2.has_value());
    EXPECT_TRUE(p_2.value().isApproxToConstant(1.4));

    // Add one more element and test the first element in that second time segment
    r3_spline.control_points_.push_back(4 * Eigen::Vector3d::Ones());
    auto const p_5{r3_spline.Evaluate(105)};
    ASSERT_TRUE(p_5.has_value());
    EXPECT_TRUE(p_5.value().isApproxToConstant(2));
}

// NOTE(Jack): The templated methods do no error checking! They depend on the time handling logic already being done,
// and if there is out of bounds access because the control points are not valid we will get a segfault here.
TEST(Spline_r3Spline, TestTemplatedEvaluate) {
    std::uint64_t const delta_t_ns{5};  // No effect when spline::DerivativeOrder::Null (raised to the zero power = 1)

    spline::Matrix3Kd const P1{{0, 1, 2, 3}, {0, 1, 2, 3}, {0, 1, 2, 3}};
    Vector3d const r3_1{
        spline::R3SplineEvaluation::Evaluate<double, spline::DerivativeOrder::Null>(P1, 0.2, delta_t_ns)};
    EXPECT_TRUE(r3_1.isApproxToConstant(1.2));

    // Shift the control points now to start at 1 and end at 4 manually by creating a new P, reflecting the last test in
    // "(Spline_r3Spline, TestEvaluate)".
    spline::Matrix3Kd const P2{{1, 2, 3, 4}, {1, 2, 3, 4}, {1, 2, 3, 4}};
    Vector3d const r3_2{spline::R3SplineEvaluation::Evaluate<double, spline::DerivativeOrder::Null>(P2, 0, delta_t_ns)};
    EXPECT_TRUE(r3_2.isApproxToConstant(2));
}

TEST(Spline_r3Spline, TestEvaluateDerivatives) {
    spline::r3Spline r3_spline{100, 5};
    for (int i{0}; i < spline::constants::order; ++i) {
        r3_spline.control_points_.push_back(i * Eigen::Vector3d::Ones());
    }

    auto const p_du{r3_spline.Evaluate(101, spline::DerivativeOrder::First)};
    ASSERT_TRUE(p_du.has_value());
    // Honestly I expected this to be one because our data is a linear line with slope one, but we are taking this
    // derivative with respect to time and not to the x-axis, therefore it is 0.2 m/ns because our time interval
    // (delta_t_ns) is 5.
    EXPECT_TRUE(p_du.value().isApproxToConstant(0.2));

    auto const p_dudu{r3_spline.Evaluate(101, spline::DerivativeOrder::Second)};
    ASSERT_TRUE(p_dudu.has_value());
    // Linear line has no acceleration.
    EXPECT_TRUE(p_dudu.value().isApproxToConstant(0));
}

// See the top of page five in [2] - the column vectors of u
TEST(Spline_r3Spline, TestCalculateUAtZero) {
    double const u_i{0};

    spline::VectorKd const u{spline::CalculateU(u_i)};
    spline::VectorKd const du{spline::CalculateU(u_i, spline::DerivativeOrder::First)};
    spline::VectorKd const dudu{spline::CalculateU(u_i, spline::DerivativeOrder::Second)};

    EXPECT_TRUE(u.isApprox(spline::VectorKd{1, 0, 0, 0}));
    EXPECT_TRUE(du.isApprox(spline::VectorKd{0, 1, 0, 0}));
    EXPECT_TRUE(dudu.isApprox(spline::VectorKd{0, 0, 2, 0}));
}

TEST(Spline_r3Spline, TestCalculate) {
    double const u_i{0.5};

    spline::VectorKd const u{spline::CalculateU(u_i)};
    spline::VectorKd const du{spline::CalculateU(u_i, spline::DerivativeOrder::First)};
    spline::VectorKd const dudu{spline::CalculateU(u_i, spline::DerivativeOrder::Second)};

    EXPECT_TRUE(u.isApprox(spline::VectorKd{1, 0.5, 0.25, 0.125}));
    EXPECT_TRUE(du.isApprox(spline::VectorKd{0, 1, 1, 0.75}));
    EXPECT_TRUE(dudu.isApprox(spline::VectorKd{0, 0, 2, 3}));
}