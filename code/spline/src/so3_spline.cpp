#include "spline/so3_spline.hpp"

#include <numeric>

#include "geometry/lie.hpp"
#include "spline/constants.hpp"
#include "spline/r3_spline.hpp"
#include "spline/types.hpp"
#include "spline/utilities.hpp"

namespace reprojection::spline {

std::optional<Vector3d> EvaluateSo3(std::uint64_t const t_ns, So3SplineState const& spline,
                                    DerivativeOrder const derivative) {
    auto const normalized_position{spline.time_handler.SplinePosition(t_ns, std::size(spline.control_points))};
    if (not normalized_position.has_value()) {
        return std::nullopt;
    }
    auto const [u_i, i]{normalized_position.value()};

    Matrix3Kd const P{Eigen::Map<const Matrix3Kd>(spline.control_points[i].data(), 3, constants::order)};

    if (derivative == DerivativeOrder::Null) {
        return So3SplineEvaluation::Evaluate<DerivativeOrder::Null>(P, u_i, spline.time_handler.delta_t_ns_);
    } else if (derivative == DerivativeOrder::First) {
        return So3SplineEvaluation::Evaluate<DerivativeOrder::First>(P, u_i, spline.time_handler.delta_t_ns_);
    } else if (derivative == DerivativeOrder::Second) {
        return So3SplineEvaluation::Evaluate<DerivativeOrder::Second>(P, u_i, spline.time_handler.delta_t_ns_);
    } else {
        throw std::runtime_error("Requested unknown derivative order from EvaluateSo3()");  // LCOV_EXCL_LINE
    }
}

std::array<Eigen::Vector3d, constants::degree> DeltaPhi(Matrix3Kd const& control_points) {
    std::array<Eigen::Vector3d, constants::degree> delta_phi;
    for (int j{0}; j < constants::degree; ++j) {
        delta_phi[j] = geometry::Log(geometry::Exp(control_points.col(j).eval()).inverse() *
                                     geometry::Exp(control_points.col(j + 1).eval()));
    }

    return delta_phi;
}

// TODO(Jack): Test
// TODO TOO MANY ARGS!
So3SplineEvaluationData So3SplineEvaluation::So3SplinePrepareEvaluation(Matrix3Kd const& control_points,
                                                                        double const u_i,
                                                                        std::uint64_t const delta_t_ns,
                                                                        DerivativeOrder const derivative) {
    std::array<Vector3d, constants::degree> const delta_phis{DeltaPhi(control_points)};

    std::vector<VectorKd> weights;
    for (int j{0}; j <= static_cast<int>(derivative); ++j) {
        VectorKd const u{CalculateU(u_i, derivative)};
        VectorKd const weight{M * u / std::pow(delta_t_ns, static_cast<int>(derivative))};
        weights.push_back(weight);
    }

    return So3SplineEvaluationData{delta_phis, weights};
}

}  // namespace reprojection::spline
