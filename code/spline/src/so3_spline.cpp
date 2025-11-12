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
    // auto const [u_i, i]{normalized_position.value()};

    if (derivative == DerivativeOrder::Null) {
        return So3SplineEvaluation::xEvaluate(t_ns, spline);
    } else if (derivative == DerivativeOrder::First) {
        return So3SplineEvaluation::xEvaluateVelocity(t_ns, spline);
    } else if (derivative == DerivativeOrder::Second) {
        return So3SplineEvaluation::xEvaluateAcceleration(t_ns, spline);
    } else {
        throw std::runtime_error("Requested unknown derivative order from EvaluateSo3()");  // LCOV_EXCL_LINE
    }
}

// WARN(Jack): Will not check that the control_points are valid to index! Depends on being called securely with an
// index from the time handler.
std::array<Eigen::Vector3d, constants::degree> DeltaPhi(std::vector<Vector3d> const& control_points,
                                                        int const segment) {
    std::array<Eigen::Vector3d, constants::degree> delta_phi;
    for (int j{0}; j < constants::degree; ++j) {
        delta_phi[j] = geometry::Log(geometry::Exp(control_points[segment + j]).inverse() *
                                     geometry::Exp(control_points[segment + j + 1]));
    }
    return delta_phi;
}

// TODO(Jack): Return so3?
std::optional<Vector3d> So3SplineEvaluation::xEvaluate(std::uint64_t const t_ns, So3SplineState const& spline) {
    auto const eval_data{So3SplineEvaluation::So3SplinePrepareEvaluation(t_ns, spline, DerivativeOrder::Null)};
    if (not eval_data) {
        return std::nullopt;
    }
    auto const [i, delta_phis, weights]{eval_data.value()};

    // TODO(Jack): Can we replace this all with a std::accumulate call?
    Matrix3d rotation{geometry::Exp(spline.control_points[i])};  // Can be in vector space?
    for (int j{0}; j < (constants::degree); ++j) {
        Matrix3d const delta_R{geometry::Exp((weights[0][j + 1] * delta_phis[j]).eval())};
        rotation = delta_R * rotation;
    }

    return geometry::Log(rotation);
}

std::optional<Vector3d> So3SplineEvaluation::xEvaluateVelocity(std::uint64_t const t_ns, So3SplineState const& spline) {
    auto const eval_data{So3SplinePrepareEvaluation(t_ns, spline, DerivativeOrder::First)};
    if (not eval_data) {
        return std::nullopt;
    }
    auto const [_, delta_phis, weights]{eval_data.value()};

    Vector3d velocity{Vector3d::Zero()};
    for (int j{0}; j < (constants::degree); ++j) {
        // Must use .eval() because of Eigen expression ambiguity
        // https://stackoverflow.com/questions/71437422/ambiguity-of-overloaded-function-taking-constant-eigen-argument
        Matrix3d const inverse_delta_R{geometry::Exp((weights[0][j + 1] * delta_phis[j]).eval()).inverse()};

        Vector3d const delta_v_j{weights[1][j + 1] * delta_phis[j]};
        velocity = delta_v_j + (inverse_delta_R * velocity);
    }

    return velocity;
}

std::optional<Vector3d> So3SplineEvaluation::xEvaluateAcceleration(std::uint64_t const t_ns,
                                                                  So3SplineState const& spline) {
    auto const eval_data{So3SplinePrepareEvaluation(t_ns, spline, DerivativeOrder::Second)};
    if (not eval_data) {
        return std::nullopt;
    }
    auto const [_, delta_phis, weights]{eval_data.value()};

    Vector3d velocity{Vector3d::Zero()};
    Vector3d acceleration{Vector3d::Zero()};
    for (int j{0}; j < (constants::degree); ++j) {
        Matrix3d const inverse_delta_R{geometry::Exp((weights[0][j + 1] * delta_phis[j]).eval()).inverse()};

        Vector3d const delta_v_j{weights[1][j + 1] * delta_phis[j]};
        velocity = delta_v_j + (inverse_delta_R * velocity);

        Vector3d const delta_a_j{weights[2][j + 1] * delta_phis[j] + velocity.cross(delta_v_j)};
        acceleration = delta_a_j + (inverse_delta_R * acceleration);
    }

    return acceleration;
}

// TODO(Jack): Test
std::optional<So3SplineEvaluationData> So3SplineEvaluation::So3SplinePrepareEvaluation(std::uint64_t const t_ns,
                                                                                       So3SplineState const& spline,
                                                                                       DerivativeOrder const order) {
    auto const normalized_position{spline.time_handler.SplinePosition(t_ns, std::size(spline.control_points))};
    if (not normalized_position.has_value()) {
        return std::nullopt;
    }
    auto const [u_i, i]{normalized_position.value()};

    std::array<Vector3d, constants::degree> const delta_phis{DeltaPhi(spline.control_points, i)};

    std::vector<VectorKd> weights;
    for (int j{0}; j <= static_cast<int>(order); ++j) {
        VectorKd const u{CalculateU(u_i, order)};
        VectorKd const weight{M * u / std::pow(spline.time_handler.delta_t_ns_, static_cast<int>(order))};
        weights.push_back(weight);
    }

    return So3SplineEvaluationData{i, delta_phis, weights};
}

}  // namespace reprojection::spline
