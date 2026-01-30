#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "spline/spline_evaluation_concept.hpp"
#include "spline/spline_state.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

// TODO(Jack): The naming in the entire package is a little unclear because we use the generic "spline" to refere to the
// C3 splines, and leave the C6 se3 spline out in lalaland. When we figure out better to optimize the se3 spline we
// should clear up this delineation.
/**
 * \brief Provides bounds checked evaluation of a spline. Will return std::nullopt if the requested evaluation time t_ns
 * is not a valid time on the spline. Defaults to evaluating the spline value unless passed a higher derivative order.
 *
 * This function provides a uniform interface for consumers to evaluate R3 or so3 splines in NON-optimization related
 * applications. This is useful for places like visualization, test data generation, etc. If you need to work with
 * autodiff capable spline evaluation functions you should look directly at the templated R3Spline::Evaluate and
 * So3Spline::Evaluate methods. These are used in the cost function and nonlinear optimization ceres related code.
 */
template <typename T_Model>
    requires CanEvaluateCubicBSplineC3<T_Model>
std::optional<Vector3d> EvaluateSpline(std::uint64_t const t_ns, CubicBSplineC3 const& spline,
                                       DerivativeOrder const derivative = DerivativeOrder::Null) {
    auto const normalized_position{spline.time_handler.SplinePosition(t_ns, std::size(spline.control_points))};
    if (not normalized_position.has_value()) {
        return std::nullopt;
    }
    auto const [u_i, i]{normalized_position.value()};

    Matrix3Kd const P{Eigen::Map<const Matrix3Kd>(spline.control_points[i].data(), 3, constants::order)};

    if (derivative == DerivativeOrder::Null) {
        return T_Model::template Evaluate<double, DerivativeOrder::Null>(P, u_i, spline.time_handler.delta_t_ns_);
    } else if (derivative == DerivativeOrder::First) {
        return T_Model::template Evaluate<double, DerivativeOrder::First>(P, u_i, spline.time_handler.delta_t_ns_);
    } else if (derivative == DerivativeOrder::Second) {
        return T_Model::template Evaluate<double, DerivativeOrder::Second>(P, u_i, spline.time_handler.delta_t_ns_);
    } else {
        throw std::runtime_error("Requested unknown derivative order from EvaluateSpline()");  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::spline
