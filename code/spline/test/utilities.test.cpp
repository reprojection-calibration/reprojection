#include "spline/utilities.hpp"

#include <gtest/gtest.h>

#include "types/eigen_types.hpp"

using namespace reprojection;

// Reference [1] Efficient Derivative Computation for B-Splines on Lie Groups
// Reference [2] Spline Fusion: A continuous-time representation for visual-inertial fusion with application to rolling
// shutter cameras

// See the top of page five in [2] - the column vectors of u
TEST(SplineUtilities, TestCalculateUAtZero) {
    double const u_i{0};

    spline::VectorKd const u{spline::CalculateU(u_i)};
    spline::VectorKd const du{spline::CalculateU(u_i, spline::DerivativeOrder::First)};
    spline::VectorKd const dudu{spline::CalculateU(u_i, spline::DerivativeOrder::Second)};

    EXPECT_TRUE(u.isApprox(spline::VectorKd{1, 0, 0, 0}));
    EXPECT_TRUE(du.isApprox(spline::VectorKd{0, 1, 0, 0}));
    EXPECT_TRUE(dudu.isApprox(spline::VectorKd{0, 0, 2, 0}));
}

TEST(SplineUtilities, TestCalculateU) {
    double const u_i{0.5};

    spline::VectorKd const u{spline::CalculateU(u_i)};
    spline::VectorKd const du{spline::CalculateU(u_i, spline::DerivativeOrder::First)};
    spline::VectorKd const dudu{spline::CalculateU(u_i, spline::DerivativeOrder::Second)};

    EXPECT_TRUE(u.isApprox(spline::VectorKd{1, 0.5, 0.25, 0.125}));
    EXPECT_TRUE(du.isApprox(spline::VectorKd{0, 1, 1, 0.75}));
    EXPECT_TRUE(dudu.isApprox(spline::VectorKd{0, 0, 2, 3}));
}

TEST(SplineUtilities, TestPolynomialCoefficients) {
    Matrix4d const polynomial_coefficients{spline::PolynomialCoefficients(4)};

    Matrix4d const gt_polynomial_coefficients{{1, 1, 1, 1}, {0, 1, 2, 3}, {0, 0, 2, 6}, {0, 0, 0, 6}};
    EXPECT_TRUE(polynomial_coefficients.isApprox(gt_polynomial_coefficients));
}

TEST(SplineUtilities, TestTimePolynomial) {
    VectorXd const result0{spline::TimePolynomial(4, 0.1, 0)};
    EXPECT_EQ(result0.rows(), 4);
    EXPECT_TRUE(result0.isApprox(Vector4d{1, 1.0 / 10, 1.0 / 100, 1.0 / 1000}));

    VectorXd const result1{spline::TimePolynomial(4, 0.1, 1)};
    EXPECT_EQ(result1.rows(), 4);
    EXPECT_TRUE(result1.isApprox(Vector4d{0, 1, 1.0 / 10, 1.0 / 100}));

    VectorXd const result2{spline::TimePolynomial(4, 0.1, 2)};
    EXPECT_EQ(result2.rows(), 4);
    EXPECT_TRUE(result2.isApprox(Vector4d{0, 0, 1, 1.0 / 10}));
}

TEST(SplineUtilities, TestBlendingMatrix) {
    MatrixXd const blender{spline::BlendingMatrix(4)};
    EXPECT_FLOAT_EQ(blender.norm(), 1.7480147);  // Heuristic

    MatrixXd const cumulative_blender{spline::CumulativeBlendingMatrix(4)};
    EXPECT_FLOAT_EQ(cumulative_blender.norm(), 1.6996732);                              // Heuristic
    EXPECT_TRUE(cumulative_blender.row(0).isApprox(Vector4d{1, 0, 0, 0}.transpose()));  // Eqn. 20 from [1]

    // Ground-truth value comes from the C matrix at the top of page five in [2]
    Matrix4d gt_cumulative_blender{{6, 0, 0, 0}, {5, 3, -3, 1}, {1, 3, 3, -2}, {0, 0, 0, 1}};
    gt_cumulative_blender /= 6;
    EXPECT_TRUE(cumulative_blender.isApprox(gt_cumulative_blender));
}

TEST(SplineUtilities, TestBinomialCoefficient) {
    // Wiki: "where it gives the number of ways, disregarding order, that k objects can be chosen from among n objects"
    EXPECT_EQ(spline::BinomialCoefficient(0, 0), 1);
    EXPECT_EQ(spline::BinomialCoefficient(10, 0), 1);
    EXPECT_EQ(spline::BinomialCoefficient(5, 5), 1);
    EXPECT_EQ(spline::BinomialCoefficient(4, 2), 6);  // Six ways to choose two elements from {1, 2, 3, 4}, namely {1,
                                                      // 2}, {1, 3}, {1, 4}, {2, 3}, {2, 4} and {3, 4}.
}

TEST(SplineUtilities, TestFactorial) {
    EXPECT_EQ(spline::Factorial(0), 1);
    EXPECT_EQ(spline::Factorial(1), 1);
    EXPECT_EQ(spline::Factorial(2), 2);
    EXPECT_EQ(spline::Factorial(3), 6);
}