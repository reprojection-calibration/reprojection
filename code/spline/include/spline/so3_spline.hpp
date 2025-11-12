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

// TODO(Jack): Do we need this struct?
struct So3SplineEvaluationData {
    std::array<Vector3d, constants::degree> delta_phis;
    std::vector<VectorKd> weights;
};

struct So3SplineEvaluation {
    static std::optional<Vector3d> xEvaluate(Matrix3Kd const& P, double const u_i, std::uint64_t const delta_t_ns);

    static std::optional<Vector3d> xEvaluateVelocity(Matrix3Kd const& P, double const u_i,
                                                     std::uint64_t const delta_t_ns);

    static std::optional<Vector3d> xEvaluateAcceleration(Matrix3Kd const& P, double const u_i,
                                                         std::uint64_t const delta_t_ns);

    static inline MatrixKK const M{CumulativeBlendingMatrix(constants::order)};

    static So3SplineEvaluationData So3SplinePrepareEvaluation(Matrix3Kd const& control_points, double const u_i,
                                                              std::uint64_t const delta_t_ns,
                                                              DerivativeOrder const derivative);
};

}  // namespace reprojection::spline
