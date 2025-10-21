#include "utilities.hpp"

#include <cmath>

namespace reprojection::spline {

// TODO(Jack): We also can calculate std::pow(delta_t_ns, derivative_order) in the constructor ahead of time if we
// find out it causes some problems.
VectorK CalculateU(double const u_i, DerivativeOrder const derivative) {
    assert(0 <= u_i and u_i < 1);

    static MatrixKK const polynomial_coefficients{
        PolynomialCoefficients(constants::k)};  // Static means it only evaluates once :)

    int const derivative_order{static_cast<int>(derivative)};
    VectorK const u{polynomial_coefficients.row(derivative_order).transpose().array() *
                    TimePolynomial(constants::k, u_i, derivative_order).array()};

    return u;
}

// For polynomial k=4
//      1 1 1 1     - zero derivative coefficients
//      0 1 2 3     - first derivative coefficients
//      0 0 2 6     - ...
//      0 0 0 6
Eigen::MatrixXd PolynomialCoefficients(int const k) {
    assert(k >= 1);

    Eigen::MatrixXd result{Eigen::MatrixXd::Zero(k, k)};
    result.row(0).setOnes();
    for (int i{1}; i < k; ++i) {
        for (int j{i}; j < k; ++j) {
            result(i, j) = (j - (i - 1)) * result(i - 1, j);
        }
    }

    return result;
}  // LCOV_EXCL_LINE

// NOTE(Jack): In the spline code in this package we sometimes we have to call it u or u_i depending if we also have the
// vector u in the same namespace.
Eigen::VectorXd TimePolynomial(int const k, double const u, int const derivative) {
    assert(k >= 1);
    assert(0 <= u and u < 1);
    assert(0 <= derivative and derivative <= k - 1);

    Eigen::VectorXd result{Eigen::VectorXd::Zero(k)};
    result(derivative) = 1;
    for (int i{1 + derivative}; i < k; ++i) {
        result(i) = result(i - 1) * u;
    }

    return result;
}  // LCOV_EXCL_LINE

Eigen::MatrixXd BlendingMatrix(int const k) {
    Eigen::MatrixXd result{Eigen::MatrixXd::Zero(k, k)};

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

Eigen::MatrixXd CumulativeBlendingMatrix(int const k) {
    Eigen::MatrixXd const blending_matrix{BlendingMatrix(k)};

    auto result{Eigen::MatrixXd::Zero(k, k).eval()};
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