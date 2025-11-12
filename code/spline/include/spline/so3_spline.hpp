#pragma once

#include <cstdint>
#include <optional>

#include "spline/time_handler.hpp"
#include "spline/utilities.hpp"
#include "types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

// USE SAME STRUCT AS R3, they are the exact same if we store the rotation as vector3d not matrix3d!
struct SO3SplineState {
    SO3SplineState(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns)
        : time_handler{t0_ns, delta_t_ns, constants::order} {}

    TimeHandler time_handler;
    std::vector<Matrix3d> control_points;  // todo change to vector
};

struct SO3SplineEvaluationData {
    int i;
    std::array<Vector3d, constants::degree> delta_phis;
    std::vector<VectorKd> weights;
};

struct SO3SplineEvaluation {
    static inline MatrixKK const M{CumulativeBlendingMatrix(constants::order)};

    static std::optional<SO3SplineEvaluationData> SO3SplinePrepareEvaluation(std::uint64_t const t_ns,
                                                                             SO3SplineState const& spline,
                                                                             DerivativeOrder const order);
};

std::optional<Matrix3d> EvaluateSO3(std::uint64_t const t_ns, SO3SplineState const& spline);

std::optional<Vector3d> EvaluateSO3Velocity(std::uint64_t const t_ns, SO3SplineState const& spline);

std::optional<Vector3d> EvaluateSO3Acceleration(std::uint64_t const t_ns, SO3SplineState const& spline);

}  // namespace reprojection::spline
