#pragma once

#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

// We are constructing the column vectors u that we multiply by C as found at the top of page five in [2] - this
// construction depends on which derivative of u we are evaluating the spline at.
// TODO(Jack): I do not like that this method requires knowledge of the spline order k, but no other method does here in
// the utilities file. Are we missing the point somewhere?
VectorKd CalculateU(double const u_i, DerivativeOrder const derivative = DerivativeOrder::Null);

// NOTE(Jack): In the spline code in this package we sometimes we have to call it u or u_i depending if we also have the
// vector u in the same namespace.
// TODO(Jack): Add using def for Eigen::Vector<T, Eigen::Dynamic>?
template <typename T>
Eigen::Vector<T, Eigen::Dynamic> TimePolynomial(int const k, T const u, int const derivative) {
    assert(k >= 1);
    assert(0 <= u and u < 1);
    assert(0 <= derivative and derivative <= k - 1);

    Eigen::Vector<T, Eigen::Dynamic> result{Eigen::Vector<T, Eigen::Dynamic>::Zero(k)};
    result(derivative) = T(1.0);
    for (int i{1 + derivative}; i < k; ++i) {
        result(i) = result(i - 1) * u;
    }

    return result;
}  // LCOV_EXCL_LINE

MatrixXd PolynomialCoefficients(int const k);

MatrixXd BlendingMatrix(int const k);

MatrixXd CumulativeBlendingMatrix(int const k);

// Note the symbol variables n and k come directly from wikipedia and are not chosen to reflect any relation to any
// other variable symbol in the spline library.
int BinomialCoefficient(int const n, int const k);

int Factorial(int const n);

}  // namespace reprojection::spline