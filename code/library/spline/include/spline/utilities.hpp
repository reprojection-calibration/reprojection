#pragma once

#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

// TODO(Jack): Should this really be an integer type valued function?
// For polynomial k=4
//      1 1 1 1     - zero derivative coefficients
//      0 1 2 3     - first derivative coefficients
//      0 0 2 6     - ...
//      0 0 0 6
template <typename T>
MatrixX<T> PolynomialCoefficients(int const k) {
    assert(k >= 1);

    MatrixX<T> result{MatrixX<T>::Zero(k, k)};
    result.row(0).setOnes();
    for (int i{1}; i < k; ++i) {
        for (int j{i}; j < k; ++j) {
            result(i, j) = static_cast<T>(j - (i - 1)) * result(i - 1, j);
        }
    }

    return result;
}

// NOTE(Jack): In the spline code in this package we sometimes we have to call it u or u_i depending if we also have the
// vector u in the same namespace.
template <typename T>
VectorX<T> TimePolynomial(int const k, T const u, int const derivative) {
    assert(k >= 1);
    assert(0 <= u and u < 1);
    assert(0 <= derivative and derivative <= k - 1);

    VectorX<T> result{VectorX<T>::Zero(k)};
    result(derivative) = static_cast<T>(1.0);
    for (int i{1 + derivative}; i < k; ++i) {
        result(i) = result(i - 1) * u;
    }

    return result;
}

// We are constructing the column vectors u that we multiply by C as found at the top of page five in [2] - this
// construction depends on which derivative of u we are evaluating the spline at.
// TODO(Jack): I do not like that this method requires knowledge of the spline order k, but no other method does here in
// the utilities file. Are we missing the point somewhere?
// TODO(Jack): We also can calculate std::pow(delta_t_ns, derivative_order) in the constructor ahead of time if we
// find out it causes some problems.
template <typename T>
VectorK<T> CalculateU(T const u_i, DerivativeOrder const derivative_order = DerivativeOrder::Null) {
    assert(0 <= u_i and u_i < 1);

    // Static means it only evaluates once :) K is a project constant which is why its ok we do this.
    static MatrixKd const polynomial_coefficients{PolynomialCoefficients<T>(K)};

    const int j{static_cast<int>(derivative_order)};
    VectorK<T> const u{polynomial_coefficients.row(j).transpose().array() * TimePolynomial<T>(K, u_i, j).array()};

    return u;
}

MatrixXd BlendingMatrix(int k);

MatrixXd CumulativeBlendingMatrix(int k);

// Note the symbol variables n and k come directly from wikipedia and are not chosen to reflect any relation to any
// other variable symbol in the spline library.
int BinomialCoefficient(int n, int k);

int Factorial(int n);

}  // namespace reprojection::spline