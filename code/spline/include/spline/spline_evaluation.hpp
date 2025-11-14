#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "spline/spline_states.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

/***
 * \brief Provides bounds checked evaluation of a spline. Will return std::nullopt if the requested evaluation time t_ns
 * is not a valid time on the spline.
 */
// TODO(Jack): Add concept requirement!
template <typename T_Model>
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
        throw std::runtime_error("Requested unknown derivative order from EvaluateR3()");  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::spline
