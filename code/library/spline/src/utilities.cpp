#include "spline/utilities.hpp"

#include <cmath>

namespace reprojection::spline {

// TODO(Jack): We also can calculate std::pow(delta_t_ns, derivative_order) in the constructor ahead of time if we
// find out it causes some problems.
VectorKd CalculateU(double const u_i, int const derivative_order) {
    assert(0 <= u_i and u_i < 1);

    static MatrixKd const polynomial_coefficients{
        PolynomialCoefficients<double>(K)};  // Static means it only evaluates once :)

    VectorKd const u{polynomial_coefficients.row(derivative_order).transpose().array() *
                     TimePolynomial<double>(K, u_i, derivative_order).array()};

    return u;
}

VectorKd CalculateU(double const u_i, DerivativeOrder const derivative) {
    int const derivative_order{static_cast<int>(derivative)};

    return CalculateU(u_i, derivative_order);
}

MatrixXd BlendingMatrix(int const k) {
    MatrixXd result{MatrixXd::Zero(k, k)};

    for (int s{0}; s < k; ++s) {
        for (int n{0}; n < k; ++n) {
            double sum_s_n{0};
            for (int l{s}; l < k; ++l) {
                sum_s_n += std::pow(-1, l - s) * BinomialCoefficient(k, l - s) * std::pow(k - 1 - l, k - 1 - n);
            }
            result(s, n) = BinomialCoefficient(k - 1, n) * sum_s_n;
        }
    }

    return result / Factorial(k - 1);
}

MatrixXd CumulativeBlendingMatrix(int const k) {
    MatrixXd const blending_matrix{BlendingMatrix(k)};

    auto result{MatrixXd::Zero(k, k).eval()};
    for (int s{0}; s < k; ++s) {
        for (int n{0}; n < k; ++n) {
            // Sum of all elements in column at or below element (l, n)
            double sum_s_n{0};
            for (int l{s}; l < k; ++l) {
                sum_s_n += blending_matrix(l, n);
            }
            result(s, n) = sum_s_n;
        }
    }

    return result;
}

// Factorial based implementation is not the fastest, but we are dealing with small values (?) so we can afford it for
// the sake of clarity https://en.wikipedia.org/wiki/Binomial_coefficient#Computing_the_value_of_binomial_coefficients
int BinomialCoefficient(int const n, int const k) {
    assert(n >= k and k >= 0);

    return Factorial(n) / (Factorial(k) * Factorial(n - k));
}

int Factorial(int const n) {
    assert(n >= 0);

    int f{1};
    for (int i{1}; i <= n; ++i) {
        f *= i;
    }

    return f;
}

}  // namespace reprojection::spline