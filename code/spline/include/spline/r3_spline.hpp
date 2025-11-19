#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"
#include "utilities.hpp"

namespace reprojection::spline {

struct R3Spline {
    template <DerivativeOrder D>
    static VectorKd B(double const u_i) {
        static int constexpr derivative_order{static_cast<int>(D)};

        static VectorKd const p{polynomial_coefficients_.row(derivative_order)};

        VectorKd const t{TimePolynomial(constants::order, u_i, derivative_order)};
        VectorKd const du{p.cwiseProduct(t)};

        return C_ * du;
    }

    template <typename T, DerivativeOrder D>
    static Vector3<T> Evaluate(Matrix3K<T> const& P, double const u_i, std::uint64_t const delta_t_ns) {
        static int constexpr derivative_order{static_cast<int>(D)};

        return P * B<D>(u_i).template cast<T>() / std::pow(delta_t_ns, derivative_order);
    }

   private:
    static inline MatrixKK const C_{BlendingMatrix(constants::order)};
    static inline MatrixKK const polynomial_coefficients_{PolynomialCoefficients(constants::order)};
};

}  // namespace reprojection::spline
