#pragma once

#include <Eigen/Dense>

#include "spline/types.hpp"

namespace reprojection::spline {

// We are constructing the column vectors u that we multiply by C as found at the top of page five in [2] - this
// construction depends on which derivative of u we are evaluating the spline at.
// TODO(Jack): I do not like that this method requires knowledge of the spline order k, but no other method does here in
// the utilities file. Are we missing the point somewhere?
VectorK CalculateU(double const u_i, DerivativeOrder const derivative = DerivativeOrder::Null);

Eigen::VectorXd TimePolynomial(int const k, double const u, int const derivative);

Eigen::MatrixXd PolynomialCoefficients(int const k);

Eigen::MatrixXd BlendingMatrix(int const k);

Eigen::MatrixXd CumulativeBlendingMatrix(int const k);

// Note the symbol variables n and k come directly from wikipedia and are not chosen to reflect any relation to any
// other variable symbol in the spline library.
int BinomialCoefficient(int const n, int const k);

int Factorial(int const n);

}  // namespace reprojection::spline