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
        static int const derivative_order{static_cast<int>(D)};
        static VectorKd const du{polynomial_coefficients.row(derivative_order).transpose()};

        // TODO(Jack): Naming, does du and t make any sense here?
        VectorKd const t{TimePolynomial(constants::order, u_i, derivative_order)};
        VectorKd const u{du.cwiseProduct(t)};

        return P * M.cast<T>() * u.cast<T>() / std::pow(delta_t_ns, derivative_order);
    }

    static inline MatrixKK const M{BlendingMatrix(constants::order)};
    static inline MatrixKK const polynomial_coefficients{PolynomialCoefficients(constants::order)};
};

// NOTE(Jack): It is a uniform spline which presupposes that all added control_points correspond to specific evenly
// spaced times.
// NOTE(Jack): We static variables in some places because the values are constant and  needed in one and only one
// method, therefore it does not make sense to make them part of the class and crowd the class scope.
class r3Spline {
   public:
    r3Spline(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns);

    std::optional<Vector3d> Evaluate(std::uint64_t const t_ns,
                                     DerivativeOrder const derivative = DerivativeOrder::Null) const;

    // TODO(Jack): Let us consider what benefit we would get from making this private at some later point
    std::vector<Vector3d> control_points_;

   private:
    TimeHandler time_handler_;
};

}  // namespace reprojection::spline
