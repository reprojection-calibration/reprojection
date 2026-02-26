#pragma once

#include <cstdint>
#include <vector>

#include "spline/constants.hpp"
#include "spline/time_handler.hpp"
#include "spline/types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::spline {

/**
 * \brief Container for the state of a cubic b-spline that has control points of size 3x1.
 *
 * The name "cubic b-spline c3" means that it is a cubic b-spline with three-dimensional control points. This works
 * conveniently for both Euclidean R3 splines and so3 rotations spline stored in the axis angle representation.
 */
struct CubicBSplineC3 {
    CubicBSplineC3(Eigen::Ref<MatrixNXd const> const& control_points, TimeHandler const& time_handler)
        : control_points_{control_points}, time_handler_{time_handler} {}

    explicit CubicBSplineC3(std::pair<MatrixNXd, TimeHandler> const& pair) : CubicBSplineC3(pair.first, pair.second) {}

    CubicBSplineC3() = default;

    Eigen::Ref<MatrixNXd const> ControlPoints() const { return control_points_; }

    Eigen::Ref<MatrixNXd> MutableControlPoints() { return control_points_; }

    TimeHandler GetTimeHandler() const { return time_handler_; }

    std::optional<std::pair<double, int>> Position(std::uint64_t const t_ns) const {
        return time_handler_.SplinePosition(t_ns, this->Size());
    }

    std::uint64_t DeltaTNs() const { return time_handler_.delta_t_ns_; }

    size_t Size() const { return control_points_.cols(); }

   private:
    MatrixNXd control_points_;
    TimeHandler time_handler_;
};

}  // namespace reprojection::spline