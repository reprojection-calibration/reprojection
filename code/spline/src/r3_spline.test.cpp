#include "r3_spline.hpp"

#include <gtest/gtest.h>

#include "constants.hpp"
#include "types.hpp"

using namespace reprojection::spline;

TEST(r3Spline, Testr3SplineInvalidEvaluateConditions) {
    // Completely empty spline
    r3Spline r3_spline{100, 5};
    EXPECT_EQ(r3_spline.Evaluate(115), std::nullopt);

    // Add four knots which means we can ask for evaluations within the one time segment at the very start of the spline
    for (int i{0}; i < constants::k; ++i) {
        r3_spline.knots_.push_back(VectorD::Zero());
    }

    EXPECT_NE(r3_spline.Evaluate(100), std::nullopt);  // Inside first time segment - valid
    EXPECT_EQ(r3_spline.Evaluate(105), std::nullopt);  // Outside first time segment - invalid

    // Add one more knot to see that we can now do a valid evaluation in the second time segment
    r3_spline.knots_.push_back(VectorD::Zero());
    EXPECT_NE(r3_spline.Evaluate(105), std::nullopt);
}

TEST(r3Spline, Testr3SplineEvaluate) {
    // Completely empty spline
    r3Spline r3_spline{100, 5};
    for (int i{0}; i < constants::k; ++i) {
        r3_spline.knots_.push_back(i * VectorD::Ones());
    }

    // Test three elements in the first and only valid time segment
    // WARN(Jack): My first intuition is that there is an off by one error. I honestly expected p_0 to be zero not one.
    // But maybe this is correct and does not match with online reference resources simply because [1] uses a different
    // convention than most people use and I just do not understand yet.
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
    r3_spline.knots_.push_back(4 * VectorD::Ones());
    auto const p_5{r3_spline.Evaluate(105)};
    ASSERT_TRUE(p_5.has_value());
    EXPECT_TRUE(p_5.value().isApproxToConstant(2));
}

TEST(r3Spline, Testr3SplineEvaluateDerivatives) {
    // Completely empty spline
    r3Spline r3_spline{100, 5};
    for (int i{0}; i < constants::k; ++i) {
        r3_spline.knots_.push_back(i * VectorD::Ones());
    }

    auto const p_du{r3_spline.Evaluate(101, DerivativeOrder::First)};
    ASSERT_TRUE(p_du.has_value());
    // Honestly I expected this to be one because our data is a linear line with slope one, but we are taking this
    // derivative with respect to time and not to the x-axis, therefore it is 0.2 m/ns because our time interval
    // (delta_t_ns) is 5.
    EXPECT_TRUE(p_du.value().isApproxToConstant(0.2));

    auto const p_dudu{r3_spline.Evaluate(101, DerivativeOrder::Second)};
    ASSERT_TRUE(p_dudu.has_value());
    // Linear line has no acceleration.
    EXPECT_TRUE(p_dudu.value().isApproxToConstant(0));
}

// See the top of page five in [2] - the column vectors of u
TEST(r3Spline, Testr3SplineCalculateUAtZero) {
    double const u_i{0};

    VectorK const u{r3Spline::CalculateU(u_i)};
    VectorK const du{r3Spline::CalculateU(u_i, DerivativeOrder::First)};
    VectorK const dudu{r3Spline::CalculateU(u_i, DerivativeOrder::Second)};

    EXPECT_TRUE(u.isApprox(VectorK{1, 0, 0, 0}));
    EXPECT_TRUE(du.isApprox(VectorK{0, 1, 0, 0}));
    EXPECT_TRUE(dudu.isApprox(VectorK{0, 0, 2, 0}));
}

TEST(r3Spline, Testr3SplineCalculate) {
    double const u_i{0.5};

    VectorK const u{r3Spline::CalculateU(u_i)};
    VectorK const du{r3Spline::CalculateU(u_i, DerivativeOrder::First)};
    VectorK const dudu{r3Spline::CalculateU(u_i, DerivativeOrder::Second)};

    EXPECT_TRUE(u.isApprox(VectorK{1, 0.5, 0.25, 0.125}));
    EXPECT_TRUE(du.isApprox(VectorK{0, 1, 1, 0.75}));
    EXPECT_TRUE(dudu.isApprox(VectorK{0, 0, 2, 3}));
}