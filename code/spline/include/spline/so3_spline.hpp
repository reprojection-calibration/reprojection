#pragma once

#include <cstdint>
#include <optional>

#include "geometry/lie.hpp"
#include "spline/spline_state.hpp"
#include "spline/utilities.hpp"
#include "types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

// TODO(Jack): Test explicitly?
template <typename T>
std::array<Vector3<T>, constants::degree> DeltaPhi(Matrix3K<T> const& control_points) {
    std::array<Vector3<T>, constants::degree> delta_phi;
    for (int j{0}; j < constants::degree; ++j) {
        delta_phi[j] = geometry::Log<T>(geometry::Exp<T>(control_points.col(j)).inverse() *
                                        geometry::Exp<T>(control_points.col(j + 1)));
    }

    return delta_phi;
}

struct So3Spline {
    // NOTE(Jack): We are doing some compile time programming here with "if constexpr". The nature of the cumulative
    // b-spline means that the derivatives build up on top of each other incrementally. This resulted, in the first
    // iteration, in a lot of copy and pasted code. The structure here is basically that the null evaluation goes from 0
    // to 10, the first derivative goes from 0 to 20, and the second derivative goes from 0 to 30. However, the
    // evaluation methods are so intertwined that you cannot simply compose and call the null and first derivative
    // evaluations one after another to get the second derivative. They use intermediate results from eachother (see the
    // loop below), this makes composition not possible without repeating lots of computation.
    //
    // Our solution to this problem is to use "if constexpr" based on the DerivativeOrder template parameter D. This
    // allows us to generate all three version of the evaluate function from the same single source code. Please read
    // online to see how "if constexpr" works, it is not the right place to explain it here.
    template <typename T, DerivativeOrder D>
    static Vector3<T> Evaluate(Matrix3K<T> const& P, double const u_i, std::uint64_t const delta_t_ns) {
        std::array<Vector3<T>, constants::degree> const delta_phis{DeltaPhi(P)};

        int constexpr order{static_cast<int>(D)};
        std::array<VectorKd, order + 1> weights;  // We use an array because the required size is known at compile time
        for (int j{0}; j <= order; ++j) {
            VectorKd const u_j{CalculateU(u_i, j)};
            VectorKd const weight_j{M * u_j / std::pow(delta_t_ns, j)};

            weights[j] = weight_j;
        }

        Vector3<T> rotation{P.col(0)};
        Vector3<T> velocity{Vector3<T>::Zero()};
        Vector3<T> acceleration{Vector3<T>::Zero()};

        for (int j{0}; j < constants::degree; ++j) {
            VectorKd const& weight0{weights[0]};
            Matrix3<T> const delta_R_j{geometry::Exp<T>(T(weight0[j + 1]) * delta_phis[j])};
            rotation = geometry::Log<T>(delta_R_j * geometry::Exp<T>(rotation));

            if constexpr (D == DerivativeOrder::First or D == DerivativeOrder::Second) {
                Matrix3<T> const inverse_delta_R_j{delta_R_j.inverse()};

                VectorKd const& weight1{weights[1]};
                Vector3<T> const delta_v_j{T(weight1[j + 1]) * delta_phis[j]};
                velocity = delta_v_j + (inverse_delta_R_j * velocity);

                if constexpr (D == DerivativeOrder::Second) {
                    VectorKd const& weight2{weights[2]};
                    Vector3<T> const delta_a_j{T(weight2[j + 1]) * delta_phis[j] + velocity.cross(delta_v_j)};
                    acceleration = delta_a_j + (inverse_delta_R_j * acceleration);
                }
            }
        }

        if constexpr (D == DerivativeOrder::Null) {
            return rotation;
        } else if constexpr (D == DerivativeOrder::First) {
            return velocity;
        } else if constexpr (D == DerivativeOrder::Second) {
            return acceleration;
        } else {
            static_assert(D == DerivativeOrder::Null or D == DerivativeOrder::First or D == DerivativeOrder::Second,
                          "Unsupported DerivativeOrder in So3Spline::Evaluate()");
        }
    }

    static inline MatrixKK const M{CumulativeBlendingMatrix(constants::order)};
};

}  // namespace reprojection::spline
