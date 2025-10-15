#include "so3_spline.hpp"

#include <numeric>

#include "constants.hpp"
#include "geometry/lie.hpp"
#include "r3_spline.hpp"  // REMOVE AND USE COMMON GENERIC IMPLEMENTATION
#include "types.hpp"
#include "utilities.hpp"

namespace reprojection::spline {

// WARN(Jack): Will not check that the knots are valid to index! Depends on being called securely with an index from the
// time handler.
std::array<Eigen::Vector3d, constants::k - 1> DeltaPhi(std::vector<Eigen::Matrix3d> const& knots, int const segment) {
    std::array<Eigen::Vector3d, constants::k - 1> delta_phi;
    for (int j{0}; j < (constants::k - 1); ++j) {
        delta_phi[j] = geometry::Log(knots[segment + j].inverse() * knots[segment + j + 1]);
    }
    return delta_phi;
}

So3Spline::So3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns)
    : time_handler_{t0_ns, delta_t_ns, constants::k}, M_{CumulativeBlendingMatrix(constants::k)} {}

std::optional<Eigen::Matrix3d> So3Spline::Evaluate(uint64_t const t_ns) const {
    auto const normalized_position{time_handler_.SplinePosition(t_ns, std::size(knots_))};
    if (not normalized_position.has_value()) {
        return std::nullopt;
    }
    auto const [u_i, i]{normalized_position.value()};

    // TODO(Jack): Use common generic method one! Pay attention to how we use the constants here though! If we will
    // always be the same dimension for both position and rotation maybe that simplifies things.
    VectorK const u0{r3Spline::CalculateU(u_i, DerivativeOrder::Null)};
    VectorK const weight0{M_ * u0};

    // TODO(Jack): What is really the right size for all of these?
    // TODO(Jack): Is it possible or worth it to functionalize the velocity calculation?
    std::array<Eigen::Vector3d, constants::k - 1> const delta_phis{DeltaPhi(knots_, i)};

    // TODO(Jack): Can we replace this all with a std::accumulate call?
    Eigen::Matrix3d rotation{knots_[i]};
    for (int j{0}; j < (constants::k - 1); ++j) {
        Eigen::Matrix3d const delta_R{geometry::Exp((weight0[j + 1] * delta_phis[j]).eval())};
        rotation = delta_R * rotation;
    }

    return rotation;
}

// TODO(Jack): We could return matrices from all these by returning skew symmetric matrices, but I am not sure if that
// makes sense yet :)
std::optional<Eigen::Vector3d> So3Spline::EvaluateVelocity(uint64_t const t_ns) const {
    auto const normalized_position{time_handler_.SplinePosition(t_ns, std::size(knots_))};
    if (not normalized_position.has_value()) {
        return std::nullopt;
    }
    auto const [u_i, i]{normalized_position.value()};

    VectorK const u0{r3Spline::CalculateU(u_i, DerivativeOrder::Null)};
    VectorK const weight0{M_ * u0};
    VectorK const u1{r3Spline::CalculateU(u_i, DerivativeOrder::First)};
    VectorK const weight1{M_ * u1 / std::pow(time_handler_.delta_t_ns_, static_cast<int>(DerivativeOrder::First))};

    std::array<Eigen::Vector3d, constants::k - 1> const delta_phis{DeltaPhi(knots_, i)};

    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    for (int j{0}; j < (constants::k - 1); ++j) {
        // Must use .eval() because of Eigen expression ambiguity
        // https://stackoverflow.com/questions/71437422/ambiguity-of-overloaded-function-taking-constant-eigen-argument
        Eigen::Matrix3d const inverse_delta_R{geometry::Exp((weight0[j + 1] * delta_phis[j]).eval()).inverse()};

        Eigen::Vector3d const delta_v_j{weight1[j + 1] * delta_phis[j]};
        velocity = delta_v_j + (inverse_delta_R * velocity);
    }

    return velocity;
}

std::optional<Eigen::Vector3d> So3Spline::EvaluateAcceleration(uint64_t const t_ns) const {
    auto const normalized_position{time_handler_.SplinePosition(t_ns, std::size(knots_))};
    if (not normalized_position.has_value()) {
        return std::nullopt;
    }
    auto const [u_i, i]{normalized_position.value()};

    VectorK const u0{r3Spline::CalculateU(u_i, DerivativeOrder::Null)};
    VectorK const weight0{M_ * u0};
    VectorK const u1{r3Spline::CalculateU(u_i, DerivativeOrder::First)};
    VectorK const weight1{M_ * u1 / std::pow(time_handler_.delta_t_ns_, static_cast<int>(DerivativeOrder::First))};
    VectorK const u2{r3Spline::CalculateU(u_i, DerivativeOrder::Second)};
    VectorK const weight2{M_ * u2 / std::pow(time_handler_.delta_t_ns_, static_cast<int>(DerivativeOrder::Second))};

    std::array<Eigen::Vector3d, constants::k - 1> const delta_phis{DeltaPhi(knots_, i)};

    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    for (int j{0}; j < (constants::k - 1); ++j) {
        Eigen::Matrix3d const inverse_delta_R{geometry::Exp((weight0[j + 1] * delta_phis[j]).eval()).inverse()};

        Eigen::Vector3d const delta_v_j{weight1[j + 1] * delta_phis[j]};
        velocity = delta_v_j + (inverse_delta_R * velocity);

        Eigen::Vector3d const delta_a_j{weight2[j + 1] * delta_phis[j] + velocity.cross(delta_v_j)};
        acceleration = delta_a_j + (inverse_delta_R * acceleration);
    }

    return acceleration;
}

}  // namespace reprojection::spline
