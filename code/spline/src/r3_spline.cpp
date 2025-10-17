#include "spline/r3_spline.hpp"

#include "spline/constants.hpp"
#include "spline/types.hpp"
#include "utilities.hpp"

namespace reprojection::spline {

r3Spline::r3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns) : time_handler_{t0_ns, delta_t_ns, constants::k} {}

std::optional<Eigen::Vector3d> r3Spline::Evaluate(uint64_t const t_ns, DerivativeOrder const derivative) const {
    auto const normalized_position{time_handler_.SplinePosition(t_ns, std::size(knots_))};
    if (not normalized_position.has_value()) {
        return std::nullopt;
    }
    auto const [u_i, i]{normalized_position.value()};

    Matrix3K const P{Eigen::Map<const Matrix3K>(knots_[i].data(), 3, constants::k)};
    static MatrixKK const M{BlendingMatrix(constants::k)};  // Static means it only evaluates once :)
    VectorK const u{CalculateU(u_i, derivative)};

    return (P * M * u) / std::pow(time_handler_.delta_t_ns_, static_cast<int>(derivative));
}



}  // namespace reprojection::spline
