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
struct R3SplineEvaluation {
    template <typename T, DerivativeOrder D>
    static Eigen::Matrix<T, 3, 1> Evaluate(Matrix3k<T> const& P, double const u_i) {
        static int derivative_order{static_cast<int>(D)};
        static VectorKd const du{polynomial_coefficients.row(derivative_order).transpose()};

        VectorKd const t{TimePolynomial(constants::order, u_i, derivative_order)};
        VectorKd const u{du.cwiseProduct(t)};

        return P * M.cast<T>() * u.cast<T>();
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
