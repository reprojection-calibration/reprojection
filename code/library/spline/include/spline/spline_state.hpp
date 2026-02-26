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
    // TODO(Jack): Is this actually used in the wild anywhere in non-test code?
    CubicBSplineC3(Eigen::Ref<MatrixNXd const> const& control_points, TimeHandler const& time_handler);

    // TODO(Jack): Is this actually used in the wild anywhere in non-test code?
    CubicBSplineC3(std::vector<Vector3d> const& control_points, TimeHandler const& time_handler);

    // TODO REMOVE IF INITIALIZATION LOGIC CHANGES AGAIN!
    explicit CubicBSplineC3(std::pair<MatrixNXd, TimeHandler> const& pair);

    CubicBSplineC3() = default;

    Eigen::Ref<MatrixNXd const> ControlPoints() const;

    Eigen::Ref<MatrixNXd> MutableControlPoints();

    // NAMING!!!!
    // NAMING!!!!
    // NAMING!!!!
    // NAMING!!!!
    TimeHandler TimeHandler2() const { return time_handler_; }

    // Still needed?
    // Still needed?
    // Still needed?
    // Still needed?
    std::optional<std::pair<double, int>> Position(std::uint64_t const t_ns) const;

    // Still needed?
    // Still needed?
    // Still needed?
    // Still needed?
    std::uint64_t DeltaTNs() const;

    /**
     * \brief The number of control points the spline contains.
     */
    size_t Size() const;

   private:
    MatrixNXd control_points_;
    TimeHandler time_handler_;
};

}  // namespace reprojection::spline