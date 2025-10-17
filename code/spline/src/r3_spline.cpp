#include "../include/spline/r3_spline.hpp"

#include "constants.hpp"
#include "types.hpp"
#include "utilities.hpp"

namespace reprojection::spline {

r3Spline::r3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns) : time_handler_{t0_ns, delta_t_ns, constants::k} {}

std::optional<VectorD> r3Spline::Evaluate(uint64_t const t_ns, DerivativeOrder const derivative) const {
    auto const normalized_position{time_handler_.SplinePosition(t_ns, std::size(knots_))};
    if (not normalized_position.has_value()) {
        return std::nullopt;
    }
    auto const [u_i, i]{normalized_position.value()};

    MatrixDK const P{Eigen::Map<const MatrixDK>(knots_[i].data(), constants::d, constants::k)};
    static MatrixKK const M{BlendingMatrix(constants::k)};  // Static means it only evaluates once :)
    VectorK const u{r3Spline::CalculateU(u_i, derivative)};

    return (P * M * u) / std::pow(time_handler_.delta_t_ns_, static_cast<int>(derivative));
}

// We are constructing the column vectors u that we multiply by C as found at the top of page five in [2] - this
// construction depends on which derivative of u we are evaluating the spline at.
// TODO(Jack): We also can calculate std::pow(delta_t_ns, derivative_order) in the constructor ahead of time if we
// find out it causes some problems.
VectorK r3Spline::CalculateU(double const u_i, DerivativeOrder const derivative) {
    assert(0 <= u_i and u_i < 1);

    static MatrixKK const polynomial_coefficients{
        PolynomialCoefficients(constants::k)};  // Static means it only evaluates once :)

    int const derivative_order{static_cast<int>(derivative)};
    VectorK const u{polynomial_coefficients.row(derivative_order).transpose().array() *
                    TimePolynomial(constants::k, u_i, derivative_order).array()};

    return u;
}

}  // namespace reprojection::spline
