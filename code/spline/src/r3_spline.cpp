#include "spline/r3_spline.hpp"

#include "spline/utilities.hpp"
#include "spline/constants.hpp"
#include "spline/types.hpp"

namespace reprojection::spline {

r3Spline::r3Spline(std::uint64_t const t0_ns, std::uint64_t const delta_t_ns)
    : time_handler_{t0_ns, delta_t_ns, constants::order} {}

std::optional<Vector3d> r3Spline::Evaluate(std::uint64_t const t_ns, DerivativeOrder const derivative) const {
    auto const normalized_position{time_handler_.SplinePosition(t_ns, std::size(control_points_))};
    if (not normalized_position.has_value()) {
        return std::nullopt;
    }
    auto const [u_i, i]{normalized_position.value()};

    Matrix3Kd const P{Eigen::Map<const Matrix3Kd>(control_points_[i].data(), 3, constants::order)};
    static MatrixKK const M{BlendingMatrix(constants::order)};  // Static means it only evaluates once :)
    VectorKd const u{CalculateU(u_i, derivative)};

    return (P * M * u) / std::pow(time_handler_.delta_t_ns_, static_cast<int>(derivative));
}

}  // namespace reprojection::spline
