#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "spline/time_handler.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"
#include "utilities.hpp"

namespace reprojection::spline {

// TODO(Jack): Use vector/array consistently!
// TODO(Jack): What parameters actually need to be templated for ceres autodiff to work, do the parameter constraints
// need to be?
// NOTE(Jack): The templated methods do no error checking! They depend on the time handling logic already being done,
// and if there is out of bounds access because the control points are not valid we will get a segfault here.
struct R3SplineEvaluation {
    template <typename T, DerivativeOrder D>
    static Eigen::Matrix<T, 3, 1> Evaluate(Matrix3k<T> const& P, double const u_i, std::uint64_t const delta_t_ns) {
        static int constexpr derivative_order{static_cast<int>(D)};
        static VectorKd const du{polynomial_coefficients.row(derivative_order).transpose()};

        // TODO(Jack): Naming, does du and t make any sense here?
        VectorKd const t{TimePolynomial(constants::order, u_i, derivative_order)};
        VectorKd const u{du.cwiseProduct(t)};

        return P * M.cast<T>() * u.cast<T>() / std::pow(delta_t_ns, derivative_order);
    }

    static inline MatrixKK const M{BlendingMatrix(constants::order)};
    static inline MatrixKK const polynomial_coefficients{PolynomialCoefficients(constants::order)};
};

// TODO(Jack): Is this the right naming?
struct R3SplineState {
    R3SplineState(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns)
        : time_handler{t0_ns, delta_t_ns, constants::order} {}

    TimeHandler time_handler;
    std::vector<Vector3d> control_points;
};

// TODO(Jack): Do we need to/should we append the R3 here? I needed to do this so the SE3 call could find the right
// version of evaluate cause it could not from the arguments for some strange reason.
std::optional<Vector3d> EvaluateR3(std::uint64_t const t_ns, R3SplineState const& spline,
                                   DerivativeOrder const derivative = DerivativeOrder::Null);

}  // namespace reprojection::spline
