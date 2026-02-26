#include "spline/se3_spline.hpp"

#include "geometry/lie.hpp"
#include "spline/r3_spline.hpp"
#include "spline/so3_spline.hpp"
#include "spline/spline_evaluation.hpp"

namespace reprojection::spline {

Se3Spline::Se3Spline(Eigen::Ref<Matrix2NXd const> const& control_points, TimeHandler const& time_handler)
    : control_points_{control_points}, time_handler_{time_handler} {}

Se3Spline::Se3Spline(std::vector<Vector6d> const& control_points, TimeHandler const& time_handler)
    : Se3Spline(Eigen::Map<Matrix2NXd const>(control_points[0].data(), 6, std::size(control_points)), time_handler) {}

Se3Spline::Se3Spline(std::pair<Matrix2NXd, TimeHandler> const& pair) : Se3Spline(pair.first, pair.second) {}

std::optional<Vector6d> Se3Spline::Evaluate(std::uint64_t const t_ns, DerivativeOrder const derivative) const {
    auto const so3_term{EvaluateSpline<So3Spline>(this->So3(), time_handler_, t_ns, derivative)};
    auto const r3_term{EvaluateSpline<R3Spline>(this->R3(), time_handler_, t_ns, derivative)};

    if (not(so3_term.has_value() and r3_term.has_value())) {
        return std::nullopt;
    }

    Vector6d result;
    result.head<3>() = so3_term.value();
    result.tail<3>() = r3_term.value();

    return result;
}

}  // namespace reprojection::spline
