#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "spline/spline_evaluation_concept.hpp"
#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

// NOTE(Jack): Originally (i.e. the first six months of the project) EvaluateSpline only worked with the CubicBSplineC3
// type. But then we refactored the Se3Spline, and instead of it simply having two CubicBSplineC3 splines to store its
// state, one so3 and one r3, we changed it to now stores the control points directly in one 6 by X matrix alongside a
// time handler. To accommodate this we added here a version of EvaluateSpline which operates directly on a 3 by X
// control point matrix for either the so3 or r3 case.

/**
 * \brief Provides bounds checked evaluation of a spline. Will return std::nullopt if the requested evaluation time t_ns
 * is not a valid time on the spline.
 *
 * This function provides a uniform interface for consumers to evaluate R3 or so3 splines in NON-optimization related
 * applications (it has runtime branching). This is useful for places like visualization, test data generation, etc. If
 * you need to work with autodiff capable spline evaluation functions you should look directly at the templated
 * Se3Spline::EvaluatePose() static method.
 */
template <typename T_Model>
    requires CanEvaluateCubicBSplineC3<T_Model>
std::optional<Vector3d> EvaluateSpline(Eigen::Ref<MatrixNXd const> const& control_points,
                                       TimeHandler const& time_handler, std::uint64_t const t_ns,
                                       DerivativeOrder const derivative) {
    auto const normalized_position{time_handler.SplinePosition(t_ns, control_points.cols())};
    if (not normalized_position.has_value()) {
        return std::nullopt;
    }
    auto const [u_i, i]{normalized_position.value()};

    Eigen::Map<MatrixNKd const> const P{control_points.col(i).data(), constants::states, constants::order};

    if (derivative == DerivativeOrder::Null) {
        return T_Model::template Evaluate<double, DerivativeOrder::Null>(P, u_i, time_handler.delta_t_ns_);
    } else if (derivative == DerivativeOrder::First) {
        return T_Model::template Evaluate<double, DerivativeOrder::First>(P, u_i, time_handler.delta_t_ns_);
    } else if (derivative == DerivativeOrder::Second) {
        return T_Model::template Evaluate<double, DerivativeOrder::Second>(P, u_i, time_handler.delta_t_ns_);
    } else {
        throw std::runtime_error("Requested unknown derivative order from EvaluateSpline()");  // LCOV_EXCL_LINE
    }
}

template <typename T_Model>
    requires CanEvaluateCubicBSplineC3<T_Model>
std::optional<Vector3d> EvaluateSpline(CubicBSplineC3 const& spline, std::uint64_t const t_ns,
                                       DerivativeOrder const derivative) {
    return EvaluateSpline<T_Model>(spline.ControlPoints(), spline.GetTimeHandler(), t_ns, derivative);
}

}  // namespace reprojection::spline
