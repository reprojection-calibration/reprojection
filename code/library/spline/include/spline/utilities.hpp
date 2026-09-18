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
            result(i, j) = (j - (i - 1)) * result(i - 1, j);
        }
    }

    return result;
}  // LCOV_EXCL_LINE

// We are constructing the column vectors u that we multiply by C as found at the top of page five in [2] - this
// construction depends on which derivative of u we are evaluating the spline at.
// TODO(Jack): I do not like that this method requires knowledge of the spline order k, but no other method does here in
// the utilities file. Are we missing the point somewhere?
VectorKd CalculateU(double const u_i, int const derivative_order);

VectorKd CalculateU(double const u_i, DerivativeOrder const derivative = DerivativeOrder::Null);

// NOTE(Jack): In the spline code in this package we sometimes we have to call it u or u_i depending if we also have the
// vector u in the same namespace.
VectorXd TimePolynomial(int const k, double const u, int const derivative);

MatrixXd BlendingMatrix(int const k);

MatrixXd CumulativeBlendingMatrix(int const k);

// Note the symbol variables n and k come directly from wikipedia and are not chosen to reflect any relation to any
// other variable symbol in the spline library.
int BinomialCoefficient(int const n, int const k);

int Factorial(int const n);

}  // namespace reprojection::spline