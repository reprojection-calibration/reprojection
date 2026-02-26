#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "spline/spline_evaluation_concept.hpp"
#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

// TODO(Jack): The naming in the entire package is a little unclear because we use the generic "spline" to refers to the
// C3 splines, and leave the C6 se3 spline out in lalaland. When we figure out better to optimize the se3 spline we
// should clear up this delineation.
/**
 * \brief Provides bounds checked evaluation of a spline. Will return std::nullopt if the requested evaluation time t_ns
 * is not a valid time on the spline. Defaults to evaluating the spline value unless passed a higher derivative order.
 *
 * This function provides a uniform interface for consumers to evaluate R3 or so3 splines in NON-optimization related
 * applications (it has runtime branching). This is useful for places like visualization, test data generation, etc. If
 * you need to work with autodiff capable spline evaluation functions you should look directly at the templated
 * R3Spline::Evaluate and So3Spline::Evaluate methods. These are used in the cost function and nonlinear optimization
 * ceres related code.
 */
// TODO TEST
// TODO TEST
// TODO TEST
template <typename T_Model>
    requires CanEvaluateCubicBSplineC3<T_Model>
std::optional<Vector3d> EvaluateSpline(Eigen::Ref<MatrixNXd const> const& control_points,
                                       TimeHandler const& time_handler, std::uint64_t const t_ns,
                                       DerivativeOrder const derivative) {
    // TODO(Jack): Do we really need the spline .Size() method anymore?
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
    return EvaluateSpline<T_Model>(spline.ControlPoints(), spline.TimeHandler2(), t_ns, derivative);
}

}  // namespace reprojection::spline
