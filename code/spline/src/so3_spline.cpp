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
        return So3SplineEvaluation::xEvaluate(P, u_i, spline.time_handler.delta_t_ns_);
    } else if (derivative == DerivativeOrder::First) {
        return So3SplineEvaluation::xEvaluateVelocity(P, u_i, spline.time_handler.delta_t_ns_);
    } else if (derivative == DerivativeOrder::Second) {
        return So3SplineEvaluation::xEvaluateAcceleration(P, u_i, spline.time_handler.delta_t_ns_);
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

// TODO(Jack): Return so3?
std::optional<Vector3d> So3SplineEvaluation::xEvaluate(Matrix3Kd const& P, double const u_i,
                                                       std::uint64_t const delta_t_ns) {
    auto const [delta_phis,
                weights]{So3SplineEvaluation::So3SplinePrepareEvaluation(P, u_i, delta_t_ns, DerivativeOrder::Null)};

    // TODO(Jack): Can we replace this all with a std::accumulate call?
    Matrix3d rotation{geometry::Exp(P.col(0).eval())};  // Can be in vector space directly?
    for (int j{0}; j < (constants::degree); ++j) {
        Matrix3d const delta_R{geometry::Exp((weights[0][j + 1] * delta_phis[j]).eval())};
        rotation = delta_R * rotation;
    }

    return geometry::Log(rotation);
}

std::optional<Vector3d> So3SplineEvaluation::xEvaluateVelocity(Matrix3Kd const& P, double const u_i,
                                                               std::uint64_t const delta_t_ns) {
    auto const [delta_phis, weights]{So3SplinePrepareEvaluation(P, u_i, delta_t_ns, DerivativeOrder::First)};

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

std::optional<Vector3d> So3SplineEvaluation::xEvaluateAcceleration(Matrix3Kd const& P, double const u_i,
                                                                   std::uint64_t const delta_t_ns) {
    auto const [delta_phis, weights]{So3SplinePrepareEvaluation(P, u_i, delta_t_ns, DerivativeOrder::Second)};

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

// TODO(Jack): Test - only ever provided valid data!
// TODO MANY ARGS!
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
