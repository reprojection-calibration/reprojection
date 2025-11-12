#pragma once

#include <cstdint>
#include <optional>

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

struct So3SplineEvaluationData {
    int i;
    std::array<Vector3d, constants::degree> delta_phis;
    std::vector<VectorKd> weights;
};

struct So3SplineEvaluation {
    static std::optional<Vector3d> xEvaluate(std::uint64_t const t_ns, So3SplineState const& spline);

    static std::optional<Vector3d> xEvaluateVelocity(std::uint64_t const t_ns, So3SplineState const& spline);

    static std::optional<Vector3d> xEvaluateAcceleration(std::uint64_t const t_ns, So3SplineState const& spline);

    static inline MatrixKK const M{CumulativeBlendingMatrix(constants::order)};

    static So3SplineEvaluationData So3SplinePrepareEvaluation(std::uint64_t const t_ns, So3SplineState const& spline,
                                                              DerivativeOrder const order);
};

}  // namespace reprojection::spline
