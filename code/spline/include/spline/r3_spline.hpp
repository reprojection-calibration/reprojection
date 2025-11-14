#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "spline/spline_states.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"
#include "utilities.hpp"

namespace reprojection::spline {

struct R3Spline {
    template <typename T, DerivativeOrder D>
    static Vector3<T> Evaluate(Matrix3K<T> const& P, double const u_i, std::uint64_t const delta_t_ns) {
        static int constexpr derivative_order{static_cast<int>(D)};
        static VectorKd const du{polynomial_coefficients.row(derivative_order).transpose()};

        // TODO(Jack): Naming, does du and t make any sense here? Look at the original papers again!
        VectorKd const t{TimePolynomial(constants::order, u_i, derivative_order)};
        VectorKd const u{du.cwiseProduct(t)};

        return P * M.cast<T>() * u.cast<T>() / std::pow(delta_t_ns, derivative_order);
    }

    static inline MatrixKK const M{BlendingMatrix(constants::order)};
    static inline MatrixKK const polynomial_coefficients{PolynomialCoefficients(constants::order)};
};

}  // namespace reprojection::spline
