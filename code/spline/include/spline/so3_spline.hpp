#pragma once

#include <cstdint>
#include <optional>

#include "geometry/lie.hpp"
#include "spline/time_handler.hpp"
#include "spline/utilities.hpp"
#include "types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

// USE SAME STRUCT AS R3, they are the exact same if we store the rotation as vector3d not matrix3d!
struct So3SplineState {
    So3SplineState(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns)
        : time_handler{t0_ns, delta_t_ns, constants::order} {}

    TimeHandler time_handler;
    std::vector<Vector3d> control_points;
};

// TODO(Jack): Can we just name it Evaluate and depend on function overloading depending on the args?
std::optional<Vector3d> EvaluateSo3(std::uint64_t const t_ns, So3SplineState const& spline,
                                    DerivativeOrder const derivative = DerivativeOrder::Null);

// TODO(Jack): Test explicitly?
std::array<Eigen::Vector3d, constants::degree> DeltaPhi(Matrix3Kd const& control_points);

struct So3SplineEvaluation {
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
    template <DerivativeOrder D>
    static Vector3d Evaluate(Matrix3Kd const& P, double const u_i, std::uint64_t const delta_t_ns) {
        std::array<Vector3d, constants::degree> const delta_phis{DeltaPhi(P)};

        int constexpr order{static_cast<int>(D)};
        std::array<VectorKd, order + 1> weights;  // We use an array because the required size is known at compile time
        for (int j{0}; j <= order; ++j) {
            VectorKd const u_j{CalculateU(u_i, j)};
            VectorKd const weight_j{M * u_j / std::pow(delta_t_ns, j)};

            weights[j] = weight_j;
        }

        Vector3d rotation{Vector3d::Zero()};
        Vector3d velocity{Vector3d::Zero()};
        Vector3d acceleration{Vector3d::Zero()};

        for (int j{0}; j < constants::degree; ++j) {
            Matrix3d const delta_R_j{geometry::Exp((weights[0][j + 1] * delta_phis[j]).eval())};
            rotation = geometry::Log(delta_R_j * geometry::Exp(rotation));

            if constexpr (D == DerivativeOrder::First or D == DerivativeOrder::Second) {
                Matrix3d const inverse_delta_R_j{delta_R_j.inverse()};

                Vector3d const delta_v_j{weights[1][j + 1] * delta_phis[j]};
                velocity = delta_v_j + (inverse_delta_R_j * velocity);

                if constexpr (D == DerivativeOrder::Second) {
                    Vector3d const delta_a_j{weights[2][j + 1] * delta_phis[j] + velocity.cross(delta_v_j)};
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
                          "Unsupported DerivativeOrder in So3SplineEvaluation::Evaluate()");
        }
    }

    static inline MatrixKK const M{CumulativeBlendingMatrix(constants::order)};
};

}  // namespace reprojection::spline
