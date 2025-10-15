#include "utilities.hpp"

#include <gtest/gtest.h>

using namespace reprojection::spline;

// Reference [1] Efficient Derivative Computation for B-Splines on Lie Groups
// Reference [2] Spline Fusion: A continuous-time representation for visual-inertial fusion with application to rolling
// shutter cameras

TEST(Utilities, TestNormalizedSegmentTime) {
    // Zero elapsed time edge case
    auto const [u1, i1]{NormalizedSegmentTime(100, 100, 5)};
    EXPECT_EQ(u1, 0);
    EXPECT_EQ(i1, 0);

    // From start of fourth segment (i=3) to 40% through it (0 -> 0.4)
    auto const [u2, i2]{NormalizedSegmentTime(100, 115, 5)};
    EXPECT_EQ(u2, 0);
    EXPECT_EQ(i2, 3);
    auto const [u3, i3]{NormalizedSegmentTime(100, 116, 5)};
    EXPECT_FLOAT_EQ(u3, 0.2);
    EXPECT_EQ(i3, 3);
    auto const [u4, i4]{NormalizedSegmentTime(100, 117, 5)};
    EXPECT_FLOAT_EQ(u4, 0.4);
    EXPECT_EQ(i4, 3);
}

TEST(Utilities, TestPolynomialCoefficients) {
    Eigen::Matrix4d const polynomial_coefficients{PolynomialCoefficients(4)};

    Eigen::Matrix4d const gt_polynomial_coefficients{{1, 1, 1, 1}, {0, 1, 2, 3}, {0, 0, 2, 6}, {0, 0, 0, 6}};
    EXPECT_TRUE(polynomial_coefficients.isApprox(gt_polynomial_coefficients));
}

TEST(Utilities, TestTimePolynomial) {
    Eigen::VectorXd const result0{TimePolynomial(4, 0.1, 0)};
    EXPECT_EQ(result0.rows(), 4);
    EXPECT_TRUE(result0.isApprox(Eigen::Vector4d{1, 1.0 / 10, 1.0 / 100, 1.0 / 1000}));

    Eigen::VectorXd const result1{TimePolynomial(4, 0.1, 1)};
    EXPECT_EQ(result1.rows(), 4);
    EXPECT_TRUE(result1.isApprox(Eigen::Vector4d{0, 1, 1.0 / 10, 1.0 / 100}));

    Eigen::VectorXd const result2{TimePolynomial(4, 0.1, 2)};
    EXPECT_EQ(result2.rows(), 4);
    EXPECT_TRUE(result2.isApprox(Eigen::Vector4d{0, 0, 1, 1.0 / 10}));
}

TEST(Utilities, TestBlendingMatrix) {
    Eigen::MatrixXd const blender{BlendingMatrix(4)};
    EXPECT_FLOAT_EQ(blender.norm(), 1.7480147);  // Heuristic

    Eigen::MatrixXd const cumulative_blender{CumulativeBlendingMatrix(4)};
    EXPECT_FLOAT_EQ(cumulative_blender.norm(), 1.6996732);                                     // Heuristic
    EXPECT_TRUE(cumulative_blender.row(0).isApprox(Eigen::Vector4d{1, 0, 0, 0}.transpose()));  // Eqn. 20 from [1]

    // Ground-truth value comes from the C matrix at the top of page five in [2]
    Eigen::Matrix4d gt_cumulative_blender{{6, 0, 0, 0}, {5, 3, -3, 1}, {1, 3, 3, -2}, {0, 0, 0, 1}};
    gt_cumulative_blender /= 6;
    EXPECT_TRUE(cumulative_blender.isApprox(gt_cumulative_blender));
}

TEST(Utilities, TestBinomialCoefficient) {
    // Wiki: "where it gives the number of ways, disregarding order, that k objects can be chosen from among n objects"
    EXPECT_EQ(BinomialCoefficient(0, 0), 1);
    EXPECT_EQ(BinomialCoefficient(10, 0), 1);
    EXPECT_EQ(BinomialCoefficient(5, 5), 1);
    EXPECT_EQ(BinomialCoefficient(4, 2), 6);  // Six ways to choose two elements from {1, 2, 3, 4}, namely {1, 2}, {1,
                                              // 3}, {1, 4}, {2, 3}, {2, 4} and {3, 4}.
}

TEST(Utilities, TestFactorial) {
    EXPECT_EQ(Factorial(0), 1);
    EXPECT_EQ(Factorial(1), 1);
    EXPECT_EQ(Factorial(2), 2);
    EXPECT_EQ(Factorial(3), 6);
}