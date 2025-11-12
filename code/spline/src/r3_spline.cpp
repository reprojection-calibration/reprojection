#include "spline/r3_spline.hpp"

#include "spline/constants.hpp"
#include "spline/types.hpp"
#include "spline/utilities.hpp"

namespace reprojection::spline {

std::optional<Vector3d> EvaluateR3(std::uint64_t const t_ns, R3SplineState const& spline,
                                   DerivativeOrder const derivative) {
    auto const normalized_position{spline.time_handler.SplinePosition(t_ns, std::size(spline.control_points))};
    if (not normalized_position.has_value()) {
        return std::nullopt;
    }
    auto const [u_i, i]{normalized_position.value()};

    Matrix3Kd const P{Eigen::Map<const Matrix3Kd>(spline.control_points[i].data(), 3, constants::order)};

    if (derivative == DerivativeOrder::Null) {
        return R3SplineEvaluation::Evaluate<double, DerivativeOrder::Null>(P, u_i, spline.time_handler.delta_t_ns_);
    } else if (derivative == DerivativeOrder::First) {
        return R3SplineEvaluation::Evaluate<double, DerivativeOrder::First>(P, u_i, spline.time_handler.delta_t_ns_);
    } else if (derivative == DerivativeOrder::Second) {
        return R3SplineEvaluation::Evaluate<double, DerivativeOrder::Second>(P, u_i, spline.time_handler.delta_t_ns_);
    } else {
        throw std::runtime_error("Requested unknown derivative order from EvaluateR3()");  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::spline
