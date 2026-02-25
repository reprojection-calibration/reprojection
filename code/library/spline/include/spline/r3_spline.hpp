#pragma once

#include <cstdint>

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

        return M_ * du;
    }

    // NOTE(Jack): To be perfectly honest we passed the control points P here for the initial five months of the project
    // simple as "Matrix3K<T> const&", but when we did the great spline refactor, and started to use mapping more
    // actively I did some reading/thinking and decided to transition to using Eigen::Ref as this it the canonical way
    // to write generic code without templates in Eigen. If this really helps us or just makes life more complicated, we
    // will see.
    //
    // See here for an explanation of the possible benefits of Eigen::Ref:
    //      https://stechschulte.net/2023/11/21/eigen-ref.html
    //
    // We pass the Eigen::Ref by const& itself due to information from this link:
    //      https://stackoverflow.com/questions/21132538/correct-usage-of-the-eigenref-class
    template <typename T, DerivativeOrder D>
    static Vector3<T> Evaluate(Eigen::Ref<MatrixNK<T> const> const& P, double const u_i,
                               std::uint64_t const delta_t_ns) {
        static int constexpr derivative_order{static_cast<int>(D)};

        return P * B<D>(u_i).template cast<T>() / std::pow(delta_t_ns, derivative_order);
    }

    static inline MatrixKd const M_{BlendingMatrix(constants::order)};

   private:
    static inline MatrixKd const polynomial_coefficients_{PolynomialCoefficients(constants::order)};
};

}  // namespace reprojection::spline
