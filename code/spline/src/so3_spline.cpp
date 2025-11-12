#include "spline/so3_spline.hpp"

#include <numeric>

#include "geometry/lie.hpp"
#include "spline/constants.hpp"
#include "spline/r3_spline.hpp"
#include "spline/types.hpp"
#include "spline/utilities.hpp"

namespace reprojection::spline {

// WARN(Jack): Will not check that the control_points are valid to index! Depends on being called securely with an
// index from the time handler.
std::array<Eigen::Vector3d, constants::degree> DeltaPhi(std::vector<Eigen::Matrix3d> const& control_points,
                                                        int const segment) {
    std::array<Eigen::Vector3d, constants::degree> delta_phi;
    for (int j{0}; j < (constants::degree); ++j) {
        delta_phi[j] = geometry::Log(control_points[segment + j].inverse() * control_points[segment + j + 1]);
    }
    return delta_phi;
}

So3Spline::So3Spline(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns)
    : time_handler_{t0_ns, delta_t_ns, constants::order}, M_{CumulativeBlendingMatrix(constants::order)} {}

std::optional<Matrix3d> So3Spline::Evaluate(std::uint64_t const t_ns) const {
    auto const eval_data{SO3SplinePrepareEvaluation(t_ns, DerivativeOrder::Null)};
    if (not eval_data) {
        return std::nullopt;
    }
    auto const [i, delta_phis, weights]{eval_data.value()};

    // TODO(Jack): Can we replace this all with a std::accumulate call?
    Matrix3d rotation{control_points_[i]};
    for (int j{0}; j < (constants::degree); ++j) {
        Matrix3d const delta_R{geometry::Exp((weights[0][j + 1] * delta_phis[j]).eval())};
        rotation = delta_R * rotation;
    }

    return rotation;
}

// TODO(Jack): We could return matrices from all these by returning skew symmetric matrices, but I am not sure if that
// makes sense yet :)
std::optional<Vector3d> So3Spline::EvaluateVelocity(std::uint64_t const t_ns) const {
    auto const eval_data{SO3SplinePrepareEvaluation(t_ns, DerivativeOrder::First)};
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

std::optional<Vector3d> So3Spline::EvaluateAcceleration(std::uint64_t const t_ns) const {
    auto const eval_data{SO3SplinePrepareEvaluation(t_ns, DerivativeOrder::Second)};
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
std::optional<SO3SplineEvaluationData> So3Spline::SO3SplinePrepareEvaluation(std::uint64_t const t_ns,
                                                                             DerivativeOrder const order) const {
    auto const normalized_position{time_handler_.SplinePosition(t_ns, std::size(control_points_))};
    if (not normalized_position.has_value()) {
        return std::nullopt;
    }
    auto const [u_i, i]{normalized_position.value()};

    std::array<Vector3d, constants::degree> const delta_phis{DeltaPhi(control_points_, i)};

    std::vector<VectorKd> weights;
    for (int j{0}; j <= static_cast<int>(order); ++j) {
        VectorKd const u{CalculateU(u_i, order)};
        VectorKd const weight{M_ * u / std::pow(time_handler_.delta_t_ns_, static_cast<int>(order))};

        weights.push_back(weight);
    }

    return SO3SplineEvaluationData{i, delta_phis, weights};
}

}  // namespace reprojection::spline
