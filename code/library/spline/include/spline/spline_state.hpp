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
 * conveniently for both Euclidean R3 splines and so3 rotations spline stored in the axis angle representation. The
 * state of the spline consists of a time handler initialized at construction with the starting time and time increment
 * and the vector of control points.
 */
struct CubicBSplineC3 {
    CubicBSplineC3(Eigen::Ref<MatrixNXd const> const& control_points, TimeHandler const& time_handler)
        : control_points_{control_points}, time_handler_{time_handler} {}

    // TODO(Jack): Confirm that the mapping actually creates a copy of the data!
    CubicBSplineC3(std::vector<Vector3d> const& control_points, TimeHandler const& time_handler)
        : CubicBSplineC3(
              Eigen::Map<MatrixNXd const>(control_points[0].data(), constants::states, std::size(control_points)),
              time_handler) {}

    CubicBSplineC3() = default;

    Eigen::Ref<MatrixNXd const> ControlPoints() const { return control_points_; }

    // TODO(Jack): Does this kind of naming make sense?
    Eigen::Ref<MatrixNXd> MutableControlPoints() { return control_points_; }

    std::optional<std::pair<double, int>> Position(std::uint64_t const t_ns) const {
        return time_handler_.SplinePosition(t_ns, this->Size());
    }

    std::uint64_t DeltaTNs() const { return time_handler_.delta_t_ns_; }

    /**
     * \brief The number of control points the spline contains.
     */
    size_t Size() const { return control_points_.cols(); }

    // TODO MAKE PRIVATE

   private:
    MatrixNXd control_points_;
    TimeHandler time_handler_;
};

}  // namespace reprojection::spline